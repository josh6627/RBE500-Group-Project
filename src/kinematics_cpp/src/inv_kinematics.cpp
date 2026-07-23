#include "kinematics_cpp/srv/ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

namespace kinematics_cpp {

class InverseKinematics final : public rclcpp::Node {
  public:
    using IK = kinematics_cpp::srv::IK;

    InverseKinematics() : Node("inverse_kinematics") {
        declare_parameters();
        load_parameters();
        ik_service_ = create_service<IK>(
            "/desired_joint",
            [this](IK::Request::ConstSharedPtr request, IK::Response::SharedPtr response) {
                handle_service(request, response);
            }

        );
    }

  private:
    void declare_parameters() {
        declare_parameter<double>("link_1_length", 1.0);
        declare_parameter<double>("link_2_length", 1.0);
        declare_parameter<double>("link_3_length", 1.0);
        declare_parameter<double>("joint_3_offset", 0.04);
        declare_parameter<double>("joint_3_min", 0.0);
        declare_parameter<double>("joint_3_max", 0.5);
    }
    void load_parameters() {
        link_1_length_ = get_parameter("link_1_length").as_double();
        link_2_length_ = get_parameter("link_2_length").as_double();
        link_3_length_ = get_parameter("link_3_length").as_double();
        joint_3_offset_ = get_parameter("joint_3_offset").as_double();
        joint_3_min_ = get_parameter("joint_3_min").as_double();
        joint_3_max_ = get_parameter("joint_3_max").as_double();

        if (link_1_length_ <= 0.0 || link_2_length_ <= 0.0 || link_3_length_ <= 0.0) {
            throw std::invalid_argument("link lengths must be greater than zero");
        }
        if (joint_3_min_ > joint_3_max_) {
            throw std::invalid_argument("joint 3 minimum cannot be greater than maximum");
        }
        RCLCPP_INFO(
            get_logger(),
            "ARM Geometry: Link 1=%.3f, Link 2=%.3f, Link 3=%.3f, joint 3 offset=%.3f, prismatic "
            "range=[%.3f, %.3f]",
            link_1_length_,
            link_2_length_,
            link_3_length_,
            joint_3_offset_,
            joint_3_min_,
            joint_3_max_
        );
    }

    /**
     * @brief takes the requested target pose and calculates the desired joint solutions
     *
     * @param request geometry_msgs/pose
     * @param response includes solution count, vector of all joint solutions, and message
     */
    void handle_service(IK::Request::ConstSharedPtr request, IK::Response::SharedPtr response) {
        Eigen::Vector3d pose(
            request->target_pose.position.x,
            request->target_pose.position.y,
            request->target_pose.position.z
        );

        // print requested pose
        RCLCPP_INFO(
            this->get_logger(),
            "Desired Position: x=%f, y=%f, z=%f",
            pose.x(),
            pose.y(),
            pose.z()
        );

        const std::vector<Eigen::Vector3d> solutions = compute_inverse_kinematics(pose);

        // check if solution is empty and log the amount of solutions
        response->success = !solutions.empty();
        response->solution_count = static_cast<uint8_t>(solutions.size());
        response->joint_solutions.clear();

        // map the 3d vector solutions to the response vector
        for (const Eigen::Vector3d &solution : solutions) {
            response->joint_solutions.push_back(solution(0));
            response->joint_solutions.push_back(solution(1));
            response->joint_solutions.push_back(solution(2));
        }

        // write message depending on how many solutions were found
        if (solutions.empty()) {
            response->message = "No valid IK solutions found";
        } else if (solutions.size() == 1) {
            response->message = "One IK solution found";
        } else {
            response->message = "Two IK solutions found";
        }
    }

    /**
     * @brief computes inverse kinematics of the scara arm.
     *
     * @param pose Eigen::Vector3d
     * @return std::vector<Eigen::Vector3d>
     */
    std::vector<Eigen::Vector3d> compute_inverse_kinematics(const Eigen::Vector3d &pose) {
        std::vector<Eigen::Vector3d> solutions;

        const double Ex = pose.x();
        const double Ey = pose.y();
        const double Ez = pose.z();

        // compute d3
        double d3 = link_1_length_ - Ez - 0.04;

        // r squared
        double R_sqr = Ex * Ex + Ey * Ey;

        // if EE goal pose is zero, there is no solution
        if (R_sqr < 1e-12) {
            return solutions;
        }

        // if the prismatic target length is negative or too big target pose is not possible
        if (d3 < 0.0 || d3 > 0.5) {
            return solutions;
        }
        // compute beta
        double D1 = (R_sqr + link_2_length_ * link_2_length_ - link_3_length_ * link_3_length_) /
                    (2 * link_2_length_ * sqrt(R_sqr));
        if (std::abs(D1) > 1.0 + 1e-9) {
            return solutions;
        }
        D1 = std::clamp(D1, -1.0, 1.0);

        const double C1_abs = sqrt(1.0 - std::pow(D1, 2));

        // angles used to find theta 1
        const double alpha = std::atan2(Ey, Ex);
        const double beta_1 = std::atan2(C1_abs, D1);
        const double beta_2 = std::atan2(-C1_abs, D1);

        // theta 1
        Eigen::Vector2d theta1;
        theta1 << alpha - beta_1, alpha - beta_2;

        // Compute theta2
        double D2 = (R_sqr - link_2_length_ * link_2_length_ - link_3_length_ * link_3_length_) /
                    (2 * link_2_length_ * link_3_length_);
        if (std::abs(D2) > 1.0 + 1e-9) {
            return solutions;
        }
        D2 = std::clamp(D2, -1.0, 1.0);
        const double C2_abs = sqrt(1.0 - std::pow(D2, 2));

        Eigen::Vector2d theta2;
        theta2 << std::atan2(C2_abs, D2), std::atan2(-C2_abs, D2);

        // map angles to seperate solutions
        const Eigen::Vector3d solution_1(theta1(0), theta2(0), d3);
        const Eigen::Vector3d solution_2(theta1(1), theta2(1), d3);
        solutions.push_back(solution_1);

        // Check if both solutions are the same
        if (!solution_2.isApprox(solution_1, 1e-9)) {
            solutions.push_back(solution_2);
        }

        return solutions;
    }

    double link_1_length_{};
    double link_2_length_{};
    double link_3_length_{};
    double joint_3_offset_{};
    double joint_3_min_{};
    double joint_3_max_{};

    rclcpp::Service<IK>::SharedPtr ik_service_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematics>());
    rclcpp::shutdown();
    return 0;
}
} // namespace kinematics_cpp