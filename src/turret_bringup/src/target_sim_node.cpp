/**
 * Target Simulator Node
 *
 * Broadcasts TFs for simulated drone targets moving in figure-8 patterns.
 * Both the synthetic camera and radar sim read these TFs to observe the targets.
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class TargetSimNode : public rclcpp::Node
{
public:
    TargetSimNode() : Node("target_sim_node")
    {
        // Drone 1 parameters
        this->declare_parameter("target_range", 5.0);
        this->declare_parameter("speed", 0.3);
        this->declare_parameter("pattern_width", 1.047);   // ~60 deg horizontal sweep
        this->declare_parameter("pattern_height", 0.785);   // ~45 deg vertical sweep

        // Drone 2 parameters
        this->declare_parameter("drone2_range", 7.0);
        this->declare_parameter("drone2_speed", 0.25);
        this->declare_parameter("drone2_phase_offset", 0.52);  // ~30 deg azimuth offset
        this->declare_parameter("num_targets", 2);

        target_range_ = this->get_parameter("target_range").as_double();
        speed_ = this->get_parameter("speed").as_double();
        pattern_width_ = this->get_parameter("pattern_width").as_double();
        pattern_height_ = this->get_parameter("pattern_height").as_double();

        drone2_range_ = this->get_parameter("drone2_range").as_double();
        drone2_speed_ = this->get_parameter("drone2_speed").as_double();
        drone2_phase_offset_ = this->get_parameter("drone2_phase_offset").as_double();
        num_targets_ = this->get_parameter("num_targets").as_int();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 30.0),
            std::bind(&TargetSimNode::broadcastTargets, this));

        RCLCPP_INFO(this->get_logger(),
            "Target sim node initialized (%d drones, range1=%.1fm, range2=%.1fm)",
            num_targets_, target_range_, drone2_range_);
    }

private:
    void broadcastTargets()
    {
        auto now = this->now();

        // Drone 1: figure-8 pattern
        {
            double t = now.seconds() * speed_;
            double azimuth = (pattern_width_ / 2.0) * std::sin(t);
            double elevation = (pattern_height_ / 2.0) * std::sin(2.0 * t);

            double x = target_range_ * std::cos(elevation) * std::cos(azimuth);
            double y = target_range_ * std::cos(elevation) * std::sin(azimuth);
            double z = target_range_ * std::sin(elevation) + 0.15;

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = "base_link";
            tf_msg.child_frame_id = "target_drone_1::link";
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = z;
            tf_msg.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(tf_msg);
        }

        // Drone 2: reverse figure-8 with offset, different range/speed
        if (num_targets_ >= 2) {
            double t2 = now.seconds() * drone2_speed_;
            // Reverse direction (-sin), offset azimuth
            double azimuth = (pattern_width_ / 2.0) * std::sin(-t2) + drone2_phase_offset_;
            double elevation = (pattern_height_ / 2.0) * std::sin(2.0 * t2) * 0.7;  // smaller vertical sweep

            double x = drone2_range_ * std::cos(elevation) * std::cos(azimuth);
            double y = drone2_range_ * std::cos(elevation) * std::sin(azimuth);
            double z = drone2_range_ * std::sin(elevation) + 0.15;

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = "base_link";
            tf_msg.child_frame_id = "target_drone_2::link";
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = z;
            tf_msg.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    double target_range_;
    double speed_;
    double pattern_width_;
    double pattern_height_;
    double drone2_range_;
    double drone2_speed_;
    double drone2_phase_offset_;
    int num_targets_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetSimNode>());
    rclcpp::shutdown();
    return 0;
}
