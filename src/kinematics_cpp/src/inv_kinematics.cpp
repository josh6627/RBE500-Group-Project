#include "kinematics_cpp/srv/ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

using namespace rclcpp;
using namespace kinematics_cpp::srv;
// using Vector3d = Eigen::Matrix < double, 3, 1>;
class InverseKinematics : public Node {
  public:
    InverseKinematics() : Node("inverse_kinematics") {
        ik_service_ = this->create_service<IK>(
            "/desired_joint",
            [this](IK::Request::ConstSharedPtr request, IK::Response::SharedPtr response) {
                handle_service(request, response);
            }

        );
    }

  private:
    static constexpr double L1 = 1.0;
    static constexpr double L2 = 1.0;
    static constexpr double L3 = 1.0;

    void handle_service(IK::Request::ConstSharedPtr request, IK::Response::SharedPtr response) {
        Eigen::Vector3d pose(
            request->target_pose.position.x,
            request->target_pose.position.y,
            request->target_pose.position.z
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Desired Position: x=%f, y=%f, z=%f",
            pose.x(),
            pose.y(),
            pose.z()
        );

        const std::vector<Eigen::Vector3d> solutions = compute_inverse_kinematics(pose);

        response->success = !solutions.empty();
        response->solution_count = static_cast<uint8_t>(solutions.size());

        response->joint_solutions.clear();

        for (const Eigen::Vector3d &solution : solutions) {
            response->joint_solutions.push_back(solution(0));
            response->joint_solutions.push_back(solution(1));
            response->joint_solutions.push_back(solution(2));
        }
        if (solutions.empty()) {
            response->message = "No valid IK solutions found";
        } else if (solutions.size() == 1) {
            response->message = "One IK solution found";
        } else {
            response->message = "Two IK solutions found";
        }
    }

    std::vector<Eigen::Vector3d> compute_inverse_kinematics(const Eigen::Vector3d &pose) {
        std::vector<Eigen::Vector3d> solutions;

        const double Ex = pose.x();
        const double Ey = pose.y();
        const double Ez = pose.z();

        // compute d3
        double d3 = L1 - Ez - 0.04;

        double R_sqr = std::pow(Ex, 2) + std::pow(Ey, 2);

        const double alpha = std::atan2(Ey, Ex);

        if (R_sqr < 1e-12) {
            return solutions;
        }
        if (d3 < 0.0 || d3 > 0.5) {
            return solutions;
        }
        // compute beta
        double D1 = (R_sqr + std::pow(L2, 2) - std::pow(L3, 2)) / (2 * L2 * sqrt(R_sqr));
        if (std::abs(D1) > 1.0 + 1e-9) {
            return solutions;
        }
        D1 = std::clamp(D1, -1.0, 1.0);

        const double C1_abs = sqrt(1.0 - std::pow(D1, 2));

        const double beta_1 = std::atan2(C1_abs, D1);
        const double beta_2 = std::atan2(-C1_abs, D1);

        // theta 1
        Eigen::Vector2d theta1;
        theta1 << alpha - beta_1, alpha - beta_2;

        // Compute theta2
        double D2 = (R_sqr - std::pow(L2, 2) - std::pow(L3, 2)) / (2 * L2 * L3);
        if (std::abs(D2) > 1.0 + 1e-9) {
            return solutions;
        }
        D2 = std::clamp(D2, -1.0, 1.0);
        const double C2_abs = sqrt(1.0 - std::pow(D2, 2));

        Eigen::Vector2d theta2;
        theta2 << std::atan2(C2_abs, D2), std::atan2(-C2_abs, D2);

        const Eigen::Vector3d solution_1(theta1(0), theta2(0), d3);
        const Eigen::Vector3d solution_2(theta1(1), theta2(1), d3);
        solutions.push_back(solution_1);

        if (!solution_2.isApprox(solution_1, 1e-9)) {
            solutions.push_back(solution_2);
        }

        return solutions;
    }

    Service<IK>::SharedPtr ik_service_;
};

int main(int argc, char *argv[]) {
    init(argc, argv);
    spin(std::make_shared<InverseKinematics>());
    shutdown();
    return 0;
}