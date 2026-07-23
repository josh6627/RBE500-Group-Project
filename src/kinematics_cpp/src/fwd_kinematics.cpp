#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <vector>

using namespace geometry_msgs::msg;
using JointState = sensor_msgs::msg::JointState;
using namespace rclcpp;
class ForwardKinematics : public Node {

  public:
    ForwardKinematics() : Node("forward_kinematics") {
        joint_subscription_ = this->create_subscription<JointState>(
            "/joint_states",
            10,
            [this](JointState::ConstSharedPtr msg) { this->topic_callback(msg); }
        );
        pose_publisher_ = this->create_publisher<PoseStamped>("/current_pose", 10);
    }

  private:
    Eigen::Matrix4d dh(double theta, double d, double a, double alpha) const {
        const double ct = std::cos(theta);
        const double st = std::sin(theta);
        const double ca = std::cos(alpha);
        const double sa = std::sin(alpha);

        Eigen::Matrix4d transform;

        transform << ct, -st * ca, st * sa, a * ct, st, ct * ca, -ct * sa, a * st, 0, sa, ca, d, 0,
            0, 0, 1;

        return transform;
    }

    void topic_callback(JointState::ConstSharedPtr msg) {
        const std::vector<double> &joint_variables = msg->position;
        if (joint_variables.size() != 3) {
            RCLCPP_WARN(get_logger(), "Expected 3 joint positions");
            return;
        }
        const Eigen::Matrix4d HT_matrix = compute_fk(joint_variables);
        const PoseStamped pose_msg = ht_matrix_to_pose_msg(HT_matrix);
        pose_publisher_->publish(pose_msg);
    }

    Eigen::Matrix4d compute_fk(const std::vector<double> &joint_variables) const {
        Eigen::Matrix4d T01 = dh(joint_variables[0], 1, 1, 0);
        Eigen::Matrix4d T12 = dh(joint_variables[1], 0, 1, 0);
        Eigen::Matrix4d T23 = dh(0, -joint_variables[2] - 0.04, 0, 0);
        Eigen::Matrix4d T03 = T01 * T12 * T23;
        return T03;
    }

    PoseStamped ht_matrix_to_pose_msg(const Eigen::Matrix4d &ht_matrix) const {
        PoseStamped pose_msg = PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = ht_matrix(0, 3);
        pose_msg.pose.position.y = ht_matrix(1, 3);
        pose_msg.pose.position.z = ht_matrix(2, 3);

        const Eigen::Matrix3d rotation = ht_matrix.block<3, 3>(0, 0);
        const Eigen::Quaterniond quaternion = rotation_matrix_to_quaternion(rotation);
        pose_msg.pose.orientation.x = quaternion.x();
        pose_msg.pose.orientation.y = quaternion.y();
        pose_msg.pose.orientation.z = quaternion.z();
        pose_msg.pose.orientation.w = quaternion.w();

        return pose_msg;
    }

    Eigen::Quaterniond rotation_matrix_to_quaternion(const Eigen::Matrix3d &rotation) const {
        Eigen::Quaterniond quaternion(rotation);
        quaternion.normalize();
        return quaternion;
    }

    Subscription<JointState>::SharedPtr joint_subscription_;
    Publisher<PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char *argv[]) {
    init(argc, argv);
    spin(std::make_shared<ForwardKinematics>());
    shutdown();
    return 0;
}