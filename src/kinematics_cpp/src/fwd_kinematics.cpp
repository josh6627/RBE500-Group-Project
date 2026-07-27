#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

class ForwardKinematics : public rclcpp::Node {
    using JointState = sensor_msgs::msg::JointState;
    using PoseStamped = geometry_msgs::msg::PoseStamped;

  public:
    ForwardKinematics() : Node("forward_kinematics") {
        declare_parameters();
        load_parameters();
        joint_subscription_ = this->create_subscription<JointState>(
            "/joint_states",
            10,
            [this](JointState::ConstSharedPtr msg) { this->topic_callback(msg); }
        );
        pose_publisher_ = this->create_publisher<PoseStamped>("/current_pose", 10);
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

    /**
     * @brief DH transform calculator
     *
     * @param theta rotation in Z
     * @param d translation in Z
     * @param a translation in X
     * @param alpha translation in Y
     * @return Eigen::Matrix4d
     */
    Eigen::Matrix4d dh(double theta, double d, double a, double alpha) const {
        const double ct = std::cos(theta);
        const double st = std::sin(theta);
        const double ca = std::cos(alpha);
        const double sa = std::sin(alpha);

        Eigen::Matrix4d transform;

        // clang-format off
        transform << ct, -st * ca,  st * sa, a * ct, 
                     st,  ct * ca, -ct * sa, a * st, 
                      0,       sa,       ca,      d, 
                      0,        0,        0,      1;
        // clang-format on
        return transform;
    }

    /**
     * @brief takes a set of joint variables and returns the corresponding pose
     *
     * @param msg corresponding pose
     */
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
        const Eigen::Matrix4d HT_matrix = compute_fk(joint_variables_);
        const PoseStamped pose_msg = ht_matrix_to_pose_msg(HT_matrix);
        pose_publisher_->publish(pose_msg);
    }

    /**
     * @brief computes forward kinematics using DH method
     *
     * @param joint_variables joint angles/lengths
     * @return Eigen::Matrix4d Complete transformation from base to EE
     */
    Eigen::Matrix4d compute_fk(const std::vector<double> &joint_variables) const {
        Eigen::Matrix4d T01 = dh(joint_variables[0], link_1_length_, link_2_length_, 0);
        Eigen::Matrix4d T12 = dh(joint_variables[1], 0, link_3_length_, 0);
        Eigen::Matrix4d T23 = dh(0, -joint_variables[2] - joint_3_offset_, 0, 0);
        Eigen::Matrix4d T03 = T01 * T12 * T23;
        return T03;
    }

    /**
     * @brief Converts the HT matrix to a readable pose message
     *
     * @param ht_matrix
     * @return PoseStamped
     */
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

    /**
     * @brief converst euler angles to quaturnion
     *
     * @param rotation
     * @return Eigen::Quaterniond
     */
    Eigen::Quaterniond rotation_matrix_to_quaternion(const Eigen::Matrix3d &rotation) const {
        Eigen::Quaterniond quaternion(rotation);
        quaternion.normalize();
        return quaternion;
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
    rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematics>());
    rclcpp::shutdown();
    return 0;
}