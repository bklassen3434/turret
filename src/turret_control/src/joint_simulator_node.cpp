/**
 * Joint Simulator Node
 *
 * Simulates turret joint movement without Gazebo.
 * Receives position commands and smoothly moves joints to target positions.
 * Publishes joint states for visualization in RViz.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

class JointSimulatorNode : public rclcpp::Node
{
public:
    JointSimulatorNode() : Node("joint_simulator_node")
    {
        // Parameters
        this->declare_parameter("update_rate", 50.0);
        this->declare_parameter("max_velocity", 1.5);  // rad/s
        this->declare_parameter("yaw_limit", 3.14159);
        this->declare_parameter("pitch_limit", 1.047);

        double update_rate = this->get_parameter("update_rate").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        yaw_limit_ = this->get_parameter("yaw_limit").as_double();
        pitch_limit_ = this->get_parameter("pitch_limit").as_double();

        // Initialize joint states
        current_yaw_ = 0.0;
        current_pitch_ = 0.0;
        target_yaw_ = 0.0;
        target_pitch_ = 0.0;

        // Subscriber for position commands
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/turret_position_controller/commands", 10,
            std::bind(&JointSimulatorNode::commandCallback, this, std::placeholders::_1));

        // Publisher for joint states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // Timer for publishing joint states
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate),
            std::bind(&JointSimulatorNode::updateJoints, this));

        RCLCPP_INFO(this->get_logger(), "Joint simulator node initialized (rate: %.1f Hz)", update_rate);
    }

private:
    void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            target_yaw_ = std::clamp(msg->data[0], -yaw_limit_, yaw_limit_);
            target_pitch_ = std::clamp(msg->data[1], -pitch_limit_, pitch_limit_);
        }
    }

    void updateJoints()
    {
        auto now = this->now();

        // Calculate dt
        if (last_update_time_.nanoseconds() == 0) {
            last_update_time_ = now;
            return;
        }

        double dt = (now - last_update_time_).seconds();
        last_update_time_ = now;

        if (dt <= 0.0 || dt > 1.0) {
            dt = 0.02;
        }

        // Move joints toward targets (simple velocity-limited movement)
        double max_delta = max_velocity_ * dt;

        // Update yaw
        double yaw_error = target_yaw_ - current_yaw_;
        if (std::abs(yaw_error) > max_delta) {
            current_yaw_ += (yaw_error > 0 ? max_delta : -max_delta);
        } else {
            current_yaw_ = target_yaw_;
        }

        // Update pitch
        double pitch_error = target_pitch_ - current_pitch_;
        if (std::abs(pitch_error) > max_delta) {
            current_pitch_ += (pitch_error > 0 ? max_delta : -max_delta);
        } else {
            current_pitch_ = target_pitch_;
        }

        // Calculate velocities
        double yaw_velocity = (yaw_error > 0 ? 1 : -1) * std::min(std::abs(yaw_error) / dt, max_velocity_);
        double pitch_velocity = (pitch_error > 0 ? 1 : -1) * std::min(std::abs(pitch_error) / dt, max_velocity_);

        if (std::abs(yaw_error) < 0.001) yaw_velocity = 0.0;
        if (std::abs(pitch_error) < 0.001) pitch_velocity = 0.0;

        // Publish joint states
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = now;
        joint_state_msg.name = {"yaw_joint", "pitch_joint"};
        joint_state_msg.position = {current_yaw_, current_pitch_};
        joint_state_msg.velocity = {yaw_velocity, pitch_velocity};
        joint_state_msg.effort = {0.0, 0.0};

        joint_state_pub_->publish(joint_state_msg);
    }

    // Parameters
    double max_velocity_;
    double yaw_limit_;
    double pitch_limit_;

    // State
    double current_yaw_;
    double current_pitch_;
    double target_yaw_;
    double target_pitch_;
    rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
