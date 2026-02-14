#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>

#include "turret_msgs/msg/camera_detection.hpp"

class VisionDetectorNode : public rclcpp::Node
{
public:
    VisionDetectorNode() : Node("vision_detector_node"), detection_id_(0)
    {
        // Declare parameters
        this->declare_parameter("target_color_lower", std::vector<int>{0, 100, 100});   // HSV lower bound
        this->declare_parameter("target_color_upper", std::vector<int>{10, 255, 255});  // HSV upper bound
        this->declare_parameter("min_contour_area", 500.0);
        this->declare_parameter("max_detections_per_frame", 10);
        this->declare_parameter("camera_fov_h", 1.047);  // ~60 degrees
        this->declare_parameter("camera_fov_v", 0.785);  // ~45 degrees
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);

        // Get parameters
        auto lower = this->get_parameter("target_color_lower").as_integer_array();
        auto upper = this->get_parameter("target_color_upper").as_integer_array();
        color_lower_ = cv::Scalar(lower[0], lower[1], lower[2]);
        color_upper_ = cv::Scalar(upper[0], upper[1], upper[2]);
        min_contour_area_ = this->get_parameter("min_contour_area").as_double();
        max_detections_ = this->get_parameter("max_detections_per_frame").as_int();
        fov_h_ = this->get_parameter("camera_fov_h").as_double();
        fov_v_ = this->get_parameter("camera_fov_v").as_double();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();

        // Create publisher for detections
        detection_pub_ = this->create_publisher<turret_msgs::msg::CameraDetection>(
            "camera/detections", 10);

        // Create subscriber for camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/turret/camera/image_raw", 10,
            std::bind(&VisionDetectorNode::imageCallback, this, std::placeholders::_1));

        // Debug image publisher (optional)
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/debug_image", 1);

        RCLCPP_INFO(this->get_logger(), "Vision detector node initialized (max %d detections/frame)",
                    max_detections_);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        cv::Mat hsv, mask;

        // Convert to HSV color space
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Threshold for target color
        cv::inRange(hsv, color_lower_, color_upper_, mask);

        // Morphological operations to clean up mask
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Collect all contours exceeding min area, sorted by area (largest first)
        struct ContourInfo {
            int index;
            double area;
        };
        std::vector<ContourInfo> valid_contours;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > min_contour_area_) {
                valid_contours.push_back({static_cast<int>(i), area});
            }
        }

        // Sort by area descending
        std::sort(valid_contours.begin(), valid_contours.end(),
                  [](const ContourInfo& a, const ContourInfo& b) { return a.area > b.area; });

        // Cap at max detections
        int num_detections = std::min(static_cast<int>(valid_contours.size()), max_detections_);

        // Publish one detection per valid contour
        for (int d = 0; d < num_detections; d++) {
            int idx = valid_contours[d].index;
            double area = valid_contours[d].area;

            cv::Rect bbox = cv::boundingRect(contours[idx]);
            cv::Moments m = cv::moments(contours[idx]);

            double cx = m.m10 / m.m00;
            double cy = m.m01 / m.m00;

            // Convert pixel coordinates to bearing angles
            double pixel_offset_x = cx - image_width_ / 2.0;
            double pixel_offset_y = cy - image_height_ / 2.0;

            double azimuth = -(pixel_offset_x / image_width_) * fov_h_;   // Negative: pixel X-right -> positive Y = left
            double elevation = -(pixel_offset_y / image_height_) * fov_v_; // Negative because y increases downward

            // Publish detection
            auto detection_msg = turret_msgs::msg::CameraDetection();
            detection_msg.header = msg->header;
            detection_msg.detection_id = detection_id_++;
            detection_msg.azimuth = azimuth;
            detection_msg.elevation = elevation;
            detection_msg.pixel_x = cx;
            detection_msg.pixel_y = cy;
            detection_msg.bbox_width = bbox.width;
            detection_msg.bbox_height = bbox.height;
            detection_msg.confidence = std::min(1.0, area / 10000.0);
            detection_msg.classification = "drone";

            detection_pub_->publish(detection_msg);

            // Draw debug visualization
            cv::circle(frame, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 5, cv::Scalar(0, 255, 0), -1);
            cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "DRONE", cv::Point(bbox.x, bbox.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        // Publish debug image
        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        debug_pub_->publish(*debug_msg);
    }

    // Publishers and subscribers
    rclcpp::Publisher<turret_msgs::msg::CameraDetection>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Parameters
    cv::Scalar color_lower_;
    cv::Scalar color_upper_;
    double min_contour_area_;
    int max_detections_;
    double fov_h_;
    double fov_v_;
    int image_width_;
    int image_height_;

    // State
    uint32_t detection_id_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
