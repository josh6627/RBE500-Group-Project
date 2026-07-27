#include "kinematics_cpp/PID.hpp"
#include "kinematics_cpp/srv/set_joint_target.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace kinematics_cpp {
class PositionController final : public rclcpp::Node {
    using JointState = sensor_msgs::msg::JointState;
    using SetJointTarget = srv::SetJointTarget;

  public:
    PositionController() : Node("position_controller") {
        declare_parameters();
        load_parameters();
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                return update_pid_parameters(parameters);
            }
        );
        joint_state_subscriber_ = this->create_subscription<JointState>(
            "/joint_states",
            10,
            [this](JointState::ConstSharedPtr msg) { this->joint_position_callback(msg); }
        );

        joint_targets_service_ = this->create_service<SetJointTarget>(
            "set_joint_target",
            [this](
                SetJointTarget::Request::ConstSharedPtr request,
                SetJointTarget::Response::SharedPtr response
            ) { target_service(request, response); }
        );

        previous_time_ = this->get_clock()->now();

        effort_publishers_.reserve(effort_topics_.size());
        for (const auto &topic : effort_topics_) {
            effort_publishers_.push_back(this->create_publisher<std_msgs::msg::Float64>(topic, 10));
        }

        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(10), [this]() { control_loop(); });
    }

  private:
    /**
     * @brief Declares default parameters if none are assigned
     *
     */
    void declare_parameters() {
        declare_parameter<std::string>("model_name", "arm");
        joint_names_ = declare_parameter<std::vector<std::string>>(
            "joint_names",
            {"joint_1", "joint_2", "joint_3"}
        );
        for (const auto &joint_name : joint_names_) {
            const std::string prefix = joint_name + ".pid.";
            declare_parameter<double>(prefix + "kp", 0.0);
            declare_parameter<double>(prefix + "ki", 0.0);
            declare_parameter<double>(prefix + "kd", 0.0);
            declare_parameter<double>(prefix + "kf", 0.0);
            declare_parameter<double>(prefix + "start_i", 0.0);
        }
    }

    void load_parameters() {
        model_name_ = get_parameter("model_name").as_string();
        joint_names_ = get_parameter("joint_names").as_string_array();
        joint_positions_.assign(joint_names_.size(), 0.0);

        joint_pids_.clear();
        joint_indices_.clear();
        joint_pids_.reserve(joint_names_.size());
        effort_topics_.resize(joint_names_.size());

        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            const std::string &joint_name = joint_names_[i];
            const std::string prefix = joint_name + ".pid.";

            const double kp = get_parameter(prefix + "kp").as_double();
            const double ki = get_parameter(prefix + "ki").as_double();
            const double kd = get_parameter(prefix + "kd").as_double();
            const double kf = get_parameter(prefix + "kf").as_double();
            const double start_i = get_parameter(prefix + "start_i").as_double();

            joint_indices_[joint_name] = i;

            joint_pids_.emplace_back(kp, ki, kd, kf, start_i);
            effort_topics_[i] = "/model/" + model_name_ + "/joint/" + joint_name + "/cmd_force";
        }
    }

    rcl_interfaces::msg::SetParametersResult
    update_pid_parameters(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &parameter : parameters) {
            for (std::size_t i = 0; i < joint_names_.size(); ++i) {
                const std::string prefix = joint_names_[i] + ".pid.";

                if (parameter.get_name() == prefix + "kp" ||
                    parameter.get_name() == prefix + "ki" ||
                    parameter.get_name() == prefix + "kd" ||
                    parameter.get_name() == prefix + "kf" ||
                    parameter.get_name() == prefix + "start_i") {
                    const double kp = get_parameter(prefix + "kp").as_double();
                    const double ki = get_parameter(prefix + "ki").as_double();
                    const double kd = get_parameter(prefix + "kd").as_double();
                    const double kf = get_parameter(prefix + "kf").as_double();
                    const double start_i = get_parameter(prefix + "start_i").as_double();

                    double new_kp = kp;
                    double new_ki = ki;
                    double new_kd = kd;
                    double new_kf = kf;
                    double new_start_i = start_i;

                    if (parameter.get_name() == prefix + "kp")
                        new_kp = parameter.as_double();
                    else if (parameter.get_name() == prefix + "ki")
                        new_ki = parameter.as_double();
                    else if (parameter.get_name() == prefix + "kd")
                        new_kd = parameter.as_double();
                    else if (parameter.get_name() == prefix + "kf")
                        new_kf = parameter.as_double();
                    else if (parameter.get_name() == prefix + "start_i")
                        new_start_i = parameter.as_double();

                    joint_pids_[i].set_constants(new_kp, new_ki, new_kd, new_kf, new_start_i);

                    RCLCPP_INFO(get_logger(), "Updated PID gains for %s", joint_names_[i].c_str());
                }
            }
        }

        return result;
    }
    /**
     * @brief Callback to get current joint positions
     *
     * @param msg
     */
    void joint_position_callback(JointState::ConstSharedPtr msg) {
        bool updated_position = false;
        // Iterate through the joints in the message
        for (std::size_t msg_index = 0; msg_index < msg->name.size(); msg_index++) {
            if (msg_index >= msg->position.size()) {
                continue;
            }
            // find the map entry that corresponds to the current msg index
            const auto it = joint_indices_.find(msg->name[msg_index]);
            if (it == joint_indices_.end()) {
                continue;
            }
            // find the map index for the same joint
            const std::size_t joint_index = it->second;
            // store the position in the correct order
            joint_positions_[joint_index] = msg->position[msg_index];

            updated_position = true;
        }
        if (updated_position) {
            recevied_joint_state_ = true;
        }
    }

    void target_service(
        SetJointTarget::Request::ConstSharedPtr request,
        SetJointTarget::Response::SharedPtr response
    ) {
        if (request->joint_names.size() != request->targets.size()) {
            response->success = false;
            response->message = "Error: Joint names and Targets must have same size";
            return;
        }

        // Validate names before changing targets
        for (const auto &joint_name : request->joint_names) {
            if (joint_indices_.find(joint_name) == joint_indices_.end()) {
                response->success = false;
                response->message = "Unknown Joint: " + joint_name;
                return;
            }
        }

        for (std::size_t i = 0; i < request->joint_names.size(); i++) {
            const std::size_t joint_index = joint_indices_.at(request->joint_names[i]);
            joint_pids_[joint_index].set_target(request->targets[i]);
        }

        response->success = true;
        response->message = "Joint targets updated sucessfully";
    }

    void control_loop() {

        if (!recevied_joint_state_)
            return;

        const rclcpp::Time current_time = this->get_clock()->now();
        const double dt = (current_time - previous_time_).seconds();
        previous_time_ = current_time;

        if (dt <= 1.0e-6) {
            return;
        }
        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            std_msgs::msg::Float64 effort;
            effort.data = joint_pids_[i].compute(joint_positions_[i], dt);
            effort.data = std::clamp(effort.data, -10.0, 10.0);
            effort_publishers_[i]->publish(effort);
            // RCLCPP_INFO(get_logger(), "%s: %.2f", joint_names_[i].c_str(), joint_pids_[i].error);
        }
    }

    std::string model_name_;
    std::vector<std::string> joint_names_;
    std::vector<PID> joint_pids_;
    std::unordered_map<std::string, std::size_t> joint_indices_;
    std::vector<double> joint_positions_;
    std::vector<std::string> effort_topics_;
    rclcpp::Time previous_time_;
    bool recevied_joint_state_ = false;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Service<SetJointTarget>::SharedPtr joint_targets_service_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> effort_publishers_;
};

} // namespace kinematics_cpp

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<kinematics_cpp::PositionController>());

    rclcpp::shutdown();
    return 0;
}