#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "turret_control/pid_controller.hpp"
#include "turret_msgs/msg/tracked_target_array.hpp"

class TurretControllerNode : public rclcpp::Node
{
public:
    TurretControllerNode() : Node("turret_controller_node")
    {
        // Declare parameters
        this->declare_parameter("control_rate", 50.0);
        this->declare_parameter("yaw_kp", 2.0);
        this->declare_parameter("yaw_ki", 0.1);
        this->declare_parameter("yaw_kd", 0.5);
        this->declare_parameter("pitch_kp", 2.0);
        this->declare_parameter("pitch_ki", 0.1);
        this->declare_parameter("pitch_kd", 0.5);
        this->declare_parameter("max_velocity", 1.5);  // rad/s
        this->declare_parameter("yaw_limit", 3.14159);
        this->declare_parameter("pitch_limit", 1.047);  // ~60 degrees

        // Get parameters
        double control_rate = this->get_parameter("control_rate").as_double();
        double yaw_kp = this->get_parameter("yaw_kp").as_double();
        double yaw_ki = this->get_parameter("yaw_ki").as_double();
        double yaw_kd = this->get_parameter("yaw_kd").as_double();
        double pitch_kp = this->get_parameter("pitch_kp").as_double();
        double pitch_ki = this->get_parameter("pitch_ki").as_double();
        double pitch_kd = this->get_parameter("pitch_kd").as_double();
        double max_vel = this->get_parameter("max_velocity").as_double();
        yaw_limit_ = this->get_parameter("yaw_limit").as_double();
        pitch_limit_ = this->get_parameter("pitch_limit").as_double();

        // Configure PID controllers
        yaw_pid_.setGains(yaw_kp, yaw_ki, yaw_kd);
        yaw_pid_.setOutputLimits(-max_vel, max_vel);
        yaw_pid_.setIntegratorLimits(-1.0, 1.0);

        pitch_pid_.setGains(pitch_kp, pitch_ki, pitch_kd);
        pitch_pid_.setOutputLimits(-max_vel, max_vel);
        pitch_pid_.setIntegratorLimits(-1.0, 1.0);

        // Subscribers
        track_sub_ = this->create_subscription<turret_msgs::msg::TrackedTargetArray>(
            "tracks", 10,
            std::bind(&TurretControllerNode::trackCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TurretControllerNode::jointStateCallback, this, std::placeholders::_1));

        // Publisher for joint commands
        // Publishing position commands to position controller
        joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/turret_position_controller/commands", 10);

        // Control timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate),
            std::bind(&TurretControllerNode::controlLoop, this));

        // Initialize time tracking
        last_control_time_ = this->now();
        last_track_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Turret controller node initialized");
    }

private:
    void trackCallback(const turret_msgs::msg::TrackedTargetArray::SharedPtr msg)
    {
        if (msg->targets.empty()) {
            has_target_ = false;
            return;
        }

        // Find primary target
        for (const auto& track : msg->targets) {
            if (track.track_id == msg->primary_target_id) {
                target_azimuth_ = track.azimuth;
                target_elevation_ = track.elevation;
                target_state_ = track.state;
                has_target_ = true;
                last_track_time_ = this->now();
                first_track_received_ = true;
                return;
            }
        }

        // If primary not found, use first track
        target_azimuth_ = msg->targets[0].azimuth;
        target_elevation_ = msg->targets[0].elevation;
        target_state_ = msg->targets[0].state;
        has_target_ = true;
        last_track_time_ = this->now();
        first_track_received_ = true;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "yaw_joint") {
                current_yaw_ = msg->position[i];
            } else if (msg->name[i] == "pitch_joint") {
                current_pitch_ = msg->position[i];
            }
        }
        has_joint_state_ = true;
    }

    void controlLoop()
    {
        auto now = this->now();

        // Handle first run - times may not be initialized properly
        if (!first_control_run_) {
            last_control_time_ = now;
            first_control_run_ = true;
            return;
        }

        double dt = (now - last_control_time_).seconds();
        last_control_time_ = now;

        if (dt <= 0.0 || dt > 1.0) {
            dt = 0.02;  // Default to 50Hz
        }

        // Check if we have valid data
        if (!has_joint_state_) {
            return;
        }

        double yaw_cmd = current_yaw_;
        double pitch_cmd = current_pitch_;

        if (has_target_ && first_track_received_) {
            // Check for stale track
            double track_age = (now - last_track_time_).seconds();
            if (track_age < 0.0 || track_age > 1.0) {
                has_target_ = false;
                yaw_pid_.reset();
                pitch_pid_.reset();
            } else {
                // Simple proportional control
                double kp = 4.0;

                yaw_cmd = current_yaw_ + kp * target_azimuth_ * dt;   // Positive = turn toward ball
                pitch_cmd = current_pitch_ - kp * target_elevation_ * dt;  // Negative = tilt up

                // Apply joint limits
                yaw_cmd = std::clamp(yaw_cmd, -yaw_limit_, yaw_limit_);
                pitch_cmd = std::clamp(pitch_cmd, -pitch_limit_, pitch_limit_);
            }
        }

        // Publish joint commands
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        cmd_msg.data.push_back(yaw_cmd);
        cmd_msg.data.push_back(pitch_cmd);
        joint_cmd_pub_->publish(cmd_msg);
    }

    // PID controllers
    turret_control::PIDController yaw_pid_;
    turret_control::PIDController pitch_pid_;

    // Subscribers and publishers
    rclcpp::Subscription<turret_msgs::msg::TrackedTargetArray>::SharedPtr track_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double yaw_limit_;
    double pitch_limit_;

    // State
    bool first_control_run_ = false;
    bool first_track_received_ = false;
    bool has_target_ = false;
    bool has_joint_state_ = false;
    double target_azimuth_ = 0.0;
    double target_elevation_ = 0.0;
    uint8_t target_state_ = 0;
    double current_yaw_ = 0.0;
    double current_pitch_ = 0.0;
    rclcpp::Time last_track_time_;
    rclcpp::Time last_control_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurretControllerNode>());
    rclcpp::shutdown();
    return 0;
}
