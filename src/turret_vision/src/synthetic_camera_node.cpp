/**
 * Synthetic Camera Node
 *
 * Publishes simulated camera images by reading drone target TF positions
 * and projecting them to pixel coordinates using a pinhole camera model.
 * The target TFs are broadcast by target_sim_node.
 *
 * Renders drone silhouettes (body + arms + propellers) scaled by distance.
 * Adds realistic noise: pixel jitter, sensor noise, and random dropouts.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <random>
#include <vector>
#include <string>

class SyntheticCameraNode : public rclcpp::Node
{
public:
    SyntheticCameraNode() : Node("synthetic_camera_node")
    {
        // Parameters
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("frame_rate", 30.0);
        this->declare_parameter("drone_base_size", 30);
        this->declare_parameter("reference_distance", 5.0);
        this->declare_parameter("pixel_noise_std", 3.0);
        this->declare_parameter("image_noise_std", 15.0);
        this->declare_parameter("detection_probability", 0.95);
        this->declare_parameter("brightness_noise_std", 20.0);
        this->declare_parameter("num_targets", 2);

        width_ = this->get_parameter("image_width").as_int();
        height_ = this->get_parameter("image_height").as_int();
        double frame_rate = this->get_parameter("frame_rate").as_double();
        drone_base_size_ = this->get_parameter("drone_base_size").as_int();
        reference_distance_ = this->get_parameter("reference_distance").as_double();
        pixel_noise_std_ = this->get_parameter("pixel_noise_std").as_double();
        image_noise_std_ = this->get_parameter("image_noise_std").as_double();
        detection_probability_ = this->get_parameter("detection_probability").as_double();
        brightness_noise_std_ = this->get_parameter("brightness_noise_std").as_double();
        num_targets_ = this->get_parameter("num_targets").as_int();

        // Camera intrinsics
        fx_ = 500.0;
        fy_ = 500.0;
        cx_ = width_ / 2.0;
        cy_ = height_ / 2.0;

        // Build target frame names
        for (int i = 1; i <= num_targets_; i++) {
            target_frames_.push_back("target_drone_" + std::to_string(i) + "::link");
        }

        // Publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/turret/camera/image_raw", 10);
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/turret/camera/camera_info", 10);

        // TF listener to read target positions
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Random number generator
        std::random_device rd;
        rng_ = std::mt19937(rd());

        // Timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / frame_rate),
            std::bind(&SyntheticCameraNode::publishImage, this));

        RCLCPP_INFO(this->get_logger(),
            "Synthetic camera node initialized (%dx%d @ %.1f fps, %d targets)",
            width_, height_, frame_rate, num_targets_);
    }

private:
    void drawDrone(cv::Mat& image, double u, double v, double z, int red_val)
    {
        // Scale drone size based on distance (perspective)
        double scale = reference_distance_ / z;
        int size = static_cast<int>(drone_base_size_ * scale);
        if (size < 4) size = 4;

        int cx = static_cast<int>(u);
        int cy = static_cast<int>(v);
        cv::Scalar color(0, 0, red_val);

        // Arm length and thickness — thick enough to form one connected contour
        int arm_len = size / 2;
        int arm_thick = std::max(3, size / 4);
        int prop_radius = std::max(3, size / 4);

        // Draw arms first (thick lines so they overlap with body and propellers)
        cv::Point arm_ends[4] = {
            cv::Point(cx - arm_len, cy - arm_len),
            cv::Point(cx + arm_len, cy - arm_len),
            cv::Point(cx - arm_len, cy + arm_len),
            cv::Point(cx + arm_len, cy + arm_len),
        };
        for (int i = 0; i < 4; i++) {
            cv::line(image, cv::Point(cx, cy), arm_ends[i], color, arm_thick);
        }

        // Body: filled rectangle on top of arm intersections
        int body_w = size / 2;
        int body_h = size / 3;
        cv::rectangle(image,
            cv::Point(cx - body_w/2, cy - body_h/2),
            cv::Point(cx + body_w/2, cy + body_h/2),
            color, -1);

        // Propeller circles at arm tips — overlap with thick arm ends
        for (int i = 0; i < 4; i++) {
            cv::circle(image, arm_ends[i], prop_radius, color, -1);
        }
    }

    void publishImage()
    {
        auto now = this->now();

        // Create image with dark gray background
        cv::Mat image(height_, width_, CV_8UC3, cv::Scalar(50, 50, 50));

        // Draw some environment features (floor grid)
        for (int i = 0; i < width_; i += 50) {
            cv::line(image, cv::Point(i, height_/2), cv::Point(i, height_),
                     cv::Scalar(70, 70, 70), 1);
        }
        for (int j = height_/2; j < height_; j += 30) {
            cv::line(image, cv::Point(0, j), cv::Point(width_, j),
                     cv::Scalar(70, 70, 70), 1);
        }

        // Add per-pixel sensor noise
        if (image_noise_std_ > 0.0) {
            cv::Mat noise(height_, width_, CV_8UC3);
            cv::randn(noise, cv::Scalar(0, 0, 0),
                      cv::Scalar(image_noise_std_, image_noise_std_, image_noise_std_));
            cv::add(image, noise, image, cv::noArray(), CV_8UC3);
        }

        // Render each drone target
        for (const auto& frame : target_frames_) {
            try {
                auto transform = tf_buffer_->lookupTransform(
                    "camera_optical_frame", frame, tf2::TimePointZero);

                double x = transform.transform.translation.x;
                double y = transform.transform.translation.y;
                double z = transform.transform.translation.z;

                if (z > 0.0) {
                    // Per-drone independent dropout
                    std::uniform_real_distribution<double> uniform(0.0, 1.0);
                    if (uniform(rng_) > detection_probability_) {
                        continue;
                    }

                    // Pinhole projection
                    double u = fx_ * (x / z) + cx_;
                    double v = fy_ * (y / z) + cy_;

                    // Pixel jitter
                    if (pixel_noise_std_ > 0.0) {
                        std::normal_distribution<double> pixel_noise(0.0, pixel_noise_std_);
                        u += pixel_noise(rng_);
                        v += pixel_noise(rng_);
                    }

                    // Color variation
                    int red = 255;
                    if (brightness_noise_std_ > 0.0) {
                        std::normal_distribution<double> bright_noise(0.0, brightness_noise_std_);
                        red = std::clamp(static_cast<int>(255 + bright_noise(rng_)), 150, 255);
                    }

                    drawDrone(image, u, v, z, red);
                }
            } catch (const tf2::TransformException &ex) {
                RCLCPP_DEBUG(this->get_logger(), "Could not get transform for %s: %s",
                             frame.c_str(), ex.what());
            }
        }

        // Convert to ROS message
        std_msgs::msg::Header header;
        header.stamp = now;
        header.frame_id = "camera_optical_frame";

        auto img_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        image_pub_->publish(*img_msg);

        // Publish camera info
        auto info_msg = sensor_msgs::msg::CameraInfo();
        info_msg.header = header;
        info_msg.width = width_;
        info_msg.height = height_;
        info_msg.distortion_model = "plumb_bob";
        info_msg.k = {fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1};
        info_msg.p = {fx_, 0, cx_, 0, 0, fy_, cy_, 0, 0, 0, 1, 0};

        info_pub_->publish(info_msg);
    }

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Random number generator
    std::mt19937 rng_;

    // Image parameters
    int width_;
    int height_;
    int drone_base_size_;
    double reference_distance_;

    // Camera intrinsics
    double fx_, fy_, cx_, cy_;

    // Noise parameters
    double pixel_noise_std_;
    double image_noise_std_;
    double detection_probability_;
    double brightness_noise_std_;

    // Target frames
    int num_targets_;
    std::vector<std::string> target_frames_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyntheticCameraNode>());
    rclcpp::shutdown();
    return 0;
}
