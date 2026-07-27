#include "geometry_msgs/msg/twist.hpp"
#include "kinematics_cpp/srv/cartesian_to_joint_velocity.hpp"
#include "kinematics_cpp/srv/joint_to_cartesian_velocity.hpp"
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <stdexcept>
#include <vector>

namespace kinematics_cpp {
class VelocityKinematics final : public rclcpp::Node {
    using JointState = sensor_msgs::msg::JointState;

  public:
    VelocityKinematics() : Node("velocity_kinematics") {
        declare_parameters();
        load_parameters();
        joint_subscription_ = this->create_subscription<JointState>(
            "/joint_states",
            10,
            [this](JointState::ConstSharedPtr msg) { this->topic_callback(msg); }
        );
        joint_velocity_service_ = this->create_service<srv::CartesianToJointVelocity>(
            "/joint_velocities",
            [this](
                srv::CartesianToJointVelocity::Request::ConstSharedPtr request,
                srv::CartesianToJointVelocity::Response::SharedPtr response
            ) { joint_velocities_service(request, response); }
        );
        cartesian_velocity_service_ = this->create_service<srv::JointToCartesianVelocity>(
            "/end_effector_velocity",
            [this](
                srv::JointToCartesianVelocity::Request::ConstSharedPtr request,
                srv::JointToCartesianVelocity::Response::SharedPtr response
            ) { end_effector_velocity_service(request, response); }
        );
    }

  private:
    /**
     * @brief declares default parameter values if no values were assigned
     *
     */
    void declare_parameters() {
        declare_parameter<double>("link_1_length", 1.0);
        declare_parameter<double>("link_2_length", 1.0);
        declare_parameter<double>("link_3_length", 1.0);
        declare_parameter<double>("joint_3_offset", 0.04);
        declare_parameter<double>("joint_3_min", 0.0);
        declare_parameter<double>("joint_3_max", 0.5);
    }

    /**
     * @brief loads parameters from launch files and ensures they are valid
     *
     */
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

    void topic_callback(JointState::ConstSharedPtr msg) {
        if (msg->name.size() != msg->position.size()) {
            RCLCPP_WARN(get_logger(), "JointState names and positions have different sizes");
            return;
        }
        bool found_joint_1 = false;
        bool found_joint_2 = false;
        bool found_joint_3 = false;

        for (std::size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "joint_1") {
                joint_variables_[0] = msg->position[i];
                found_joint_1 = true;
            } else if (msg->name[i] == "joint_2") {
                joint_variables_[1] = msg->position[i];
                found_joint_2 = true;
            } else if (msg->name[i] == "joint_3") {
                joint_variables_[2] = msg->position[i];
                found_joint_3 = true;
            }
        }
        if (!found_joint_1 || !found_joint_2 || !found_joint_3) {
            RCLCPP_WARN(get_logger(), "Expected 3 joint positions");
            return;
        }
    }

    void joint_velocities_service(
        srv::CartesianToJointVelocity::Request::ConstSharedPtr request,
        srv::CartesianToJointVelocity::Response::SharedPtr response
    ) {
        const Eigen::Vector3d EE_twist(
            request->end_effector_velocity.linear.x,
            request->end_effector_velocity.linear.y,
            request->end_effector_velocity.linear.z
        );
        const Eigen::Matrix3d jacobian = calculate_jacobian(
            joint_variables_[0],
            joint_variables_[1],
            link_2_length_,
            link_3_length_
        );

        if (std::abs(jacobian.determinant()) < 1e-6) {
            response->success = false;
            response->message = "jacobian is singular or near singularity";
            return;
        }
        Eigen::Vector3d joint_velocities = jacobian.fullPivLu().solve(EE_twist);

        response->joint_velocity[0] = joint_velocities[0];
        response->joint_velocity[1] = joint_velocities[1];
        response->joint_velocity[2] = joint_velocities[2];
        response->success = true;
        response->message = "successfully calculated joint velocities";
    }

    void end_effector_velocity_service(
        srv::JointToCartesianVelocity::Request::ConstSharedPtr request,
        srv::JointToCartesianVelocity::Response::SharedPtr response
    ) {
        const Eigen::Vector3d joint_velocities(
            request->joint_velocity[0],
            request->joint_velocity[1],
            request->joint_velocity[2]
        );

        const Eigen::Matrix3d jacobian = calculate_jacobian(
            joint_variables_[0],
            joint_variables_[1],
            link_2_length_,
            link_3_length_
        );

        const Eigen::Vector3d EE_twist = jacobian * joint_velocities;

        response->cartesian_velocity.linear.x = EE_twist.x();
        response->cartesian_velocity.linear.y = EE_twist.y();
        response->cartesian_velocity.linear.z = EE_twist.z();
        response->success = true;
        response->message = "successfully calculated EE velocity";
    }

    Eigen::Matrix3d calculate_jacobian(double theta1, double theta2, double L2, double L3) {
        // clang-format off
        Eigen::Matrix3d linear_jacobian;
        linear_jacobian << -L2*sin(theta1) - L3*sin(theta1 + theta2), -L3*sin(theta1 + theta2),  0.0,
                            L2*cos(theta1) + L3*sin(theta1 + theta2),  L3*cos(theta1 + theta2),  0.0,
                                                                      0.0,                         0.0, -1.0;
        // clang-format on
        return linear_jacobian;
    }

    // Arm constants
    double link_1_length_{};
    double link_2_length_{};
    double link_3_length_{};
    double joint_3_offset_{};
    double joint_3_min_{};
    double joint_3_max_{};

    std::vector<double> joint_variables_{0.0, 0.0, 0.0};

    rclcpp::Subscription<JointState>::SharedPtr joint_subscription_;
    rclcpp::Service<srv::CartesianToJointVelocity>::SharedPtr joint_velocity_service_;
    rclcpp::Service<srv::JointToCartesianVelocity>::SharedPtr cartesian_velocity_service_;
};
} // namespace kinematics_cpp

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kinematics_cpp::VelocityKinematics>());
    rclcpp::shutdown();
    return 0;
}