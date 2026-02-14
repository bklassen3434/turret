#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <random>

#include "turret_msgs/msg/radar_detection.hpp"

class RadarSimNode : public rclcpp::Node
{
public:
    RadarSimNode() : Node("radar_sim_node"), detection_id_(0)
    {
        // Declare parameters
        this->declare_parameter("update_rate", 20.0);           // Hz
        this->declare_parameter("max_range", 50.0);             // meters
        this->declare_parameter("min_range", 0.5);              // meters
        this->declare_parameter("range_noise_std", 0.1);        // meters
        this->declare_parameter("range_rate_noise_std", 0.05);  // m/s
        this->declare_parameter("azimuth_noise_std", 0.02);     // radians (~1 degree)
        this->declare_parameter("elevation_noise_std", 0.02);   // radians
        this->declare_parameter("detection_probability", 0.95); // probability of detecting target
        this->declare_parameter("target_frame", "target_drone_1::link");
        this->declare_parameter("radar_frame", "radar_link");

        // Get parameters
        double update_rate = this->get_parameter("update_rate").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        range_noise_std_ = this->get_parameter("range_noise_std").as_double();
        range_rate_noise_std_ = this->get_parameter("range_rate_noise_std").as_double();
        azimuth_noise_std_ = this->get_parameter("azimuth_noise_std").as_double();
        elevation_noise_std_ = this->get_parameter("elevation_noise_std").as_double();
        detection_probability_ = this->get_parameter("detection_probability").as_double();
        target_frame_ = this->get_parameter("target_frame").as_string();
        radar_frame_ = this->get_parameter("radar_frame").as_string();

        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create publisher
        detection_pub_ = this->create_publisher<turret_msgs::msg::RadarDetection>(
            "radar/detections", 10);

        // Create timer for radar updates
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate),
            std::bind(&RadarSimNode::radarUpdate, this));

        // Initialize random number generator
        std::random_device rd;
        rng_ = std::mt19937(rd());

        // Store previous position for velocity calculation
        prev_position_valid_ = false;

        RCLCPP_INFO(this->get_logger(), "Radar simulation node initialized");
    }

private:
    void radarUpdate()
    {
        // Try to get transform from radar to target
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                radar_frame_, target_frame_,
                tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        // Extract position
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double z = transform.transform.translation.z;

        // Calculate range and angles
        double range = std::sqrt(x*x + y*y + z*z);
        double azimuth = std::atan2(y, x);
        double elevation = std::atan2(z, std::sqrt(x*x + y*y));

        // Check if target is within radar range
        if (range < min_range_ || range > max_range_) {
            prev_position_valid_ = false;
            return;
        }

        // Simulate detection probability (sometimes radar misses)
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        if (uniform(rng_) > detection_probability_) {
            return;  // Missed detection
        }

        // Calculate range rate (radial velocity)
        double range_rate = 0.0;
        rclcpp::Time current_time = this->now();
        if (prev_position_valid_) {
            double dt = (current_time - prev_time_).seconds();
            if (dt > 0.001) {  // Avoid division by zero
                double prev_range = std::sqrt(prev_x_*prev_x_ + prev_y_*prev_y_ + prev_z_*prev_z_);
                range_rate = (range - prev_range) / dt;
            }
        }

        // Store current position for next iteration
        prev_x_ = x;
        prev_y_ = y;
        prev_z_ = z;
        prev_time_ = current_time;
        prev_position_valid_ = true;

        // Add noise to measurements
        std::normal_distribution<double> range_noise(0.0, range_noise_std_);
        std::normal_distribution<double> rate_noise(0.0, range_rate_noise_std_);
        std::normal_distribution<double> az_noise(0.0, azimuth_noise_std_);
        std::normal_distribution<double> el_noise(0.0, elevation_noise_std_);

        double noisy_range = range + range_noise(rng_);
        double noisy_rate = range_rate + rate_noise(rng_);
        double noisy_azimuth = azimuth + az_noise(rng_);
        double noisy_elevation = elevation + el_noise(rng_);

        // Calculate SNR (simple model: decreases with range squared)
        double snr = 30.0 - 20.0 * std::log10(range / 10.0);  // ~30dB at 10m

        // Publish detection
        auto detection_msg = turret_msgs::msg::RadarDetection();
        detection_msg.header.stamp = current_time;
        detection_msg.header.frame_id = radar_frame_;
        detection_msg.detection_id = detection_id_++;
        detection_msg.range = noisy_range;
        detection_msg.range_rate = noisy_rate;
        detection_msg.azimuth = noisy_azimuth;
        detection_msg.elevation = noisy_elevation;
        detection_msg.snr = snr;
        detection_msg.confidence = std::min(1.0, snr / 30.0);

        detection_pub_->publish(detection_msg);
    }

    // Publishers and TF
    rclcpp::Publisher<turret_msgs::msg::RadarDetection>::SharedPtr detection_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double max_range_;
    double min_range_;
    double range_noise_std_;
    double range_rate_noise_std_;
    double azimuth_noise_std_;
    double elevation_noise_std_;
    double detection_probability_;
    std::string target_frame_;
    std::string radar_frame_;

    // State
    uint32_t detection_id_;
    std::mt19937 rng_;
    bool prev_position_valid_;
    double prev_x_, prev_y_, prev_z_;
    rclcpp::Time prev_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarSimNode>());
    rclcpp::shutdown();
    return 0;
}
