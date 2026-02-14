/**
 * Target Marker Node
 *
 * Publishes visualization markers for tracked targets in RViz.
 * Shows drone shapes that the turret is tracking.
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <set>
#include "turret_msgs/msg/tracked_target_array.hpp"

class TargetMarkerNode : public rclcpp::Node
{
public:
    TargetMarkerNode() : Node("target_marker_node")
    {
        track_sub_ = this->create_subscription<turret_msgs::msg::TrackedTargetArray>(
            "/tracks", 10,
            std::bind(&TargetMarkerNode::trackCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/target_markers", 10);

        RCLCPP_INFO(this->get_logger(), "Target marker node initialized");
    }

private:
    // Each drone uses a block of marker IDs: base_id + 0..9
    // 0 = body, 1-4 = arms, 5-8 = propellers, 9 = label
    static constexpr int MARKERS_PER_DRONE = 10;

    void addDroneMarkers(visualization_msgs::msg::MarkerArray& marker_array,
                         const turret_msgs::msg::TrackedTarget& target,
                         int drone_index, bool is_primary)
    {
        int base_id = drone_index * MARKERS_PER_DRONE;
        auto stamp = this->now();

        double display_range = 1.0;
        double px = display_range;
        double py = -display_range * target.azimuth;
        double pz = display_range * target.elevation;

        // Colors
        float r, g, b, a;
        if (is_primary) {
            r = 1.0; g = 0.0; b = 0.0; a = 1.0;  // bright red
        } else {
            r = 1.0; g = 0.5; b = 0.0; a = 0.8;  // orange
        }

        double body_size = 0.08;
        double arm_len = 0.12;
        double prop_size = 0.04;

        // Body cube
        {
            auto m = visualization_msgs::msg::Marker();
            m.header.frame_id = "sensor_head";
            m.header.stamp = stamp;
            m.ns = "drones";
            m.id = base_id + 0;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = px;
            m.pose.position.y = py;
            m.pose.position.z = pz;
            m.pose.orientation.w = 1.0;
            m.scale.x = body_size;
            m.scale.y = body_size * 1.5;
            m.scale.z = body_size * 0.4;
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
            m.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(m);
        }

        // 4 arms as LINE markers
        double arm_offsets[4][2] = {
            {-arm_len, -arm_len},  // front-left
            { arm_len, -arm_len},  // front-right
            {-arm_len,  arm_len},  // rear-left
            { arm_len,  arm_len},  // rear-right
        };

        for (int i = 0; i < 4; i++) {
            auto m = visualization_msgs::msg::Marker();
            m.header.frame_id = "sensor_head";
            m.header.stamp = stamp;
            m.ns = "drones";
            m.id = base_id + 1 + i;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.01;  // line width

            geometry_msgs::msg::Point p1, p2;
            p1.x = px; p1.y = py; p1.z = pz;
            p2.x = px; p2.y = py + arm_offsets[i][0]; p2.z = pz + arm_offsets[i][1];
            m.points.push_back(p1);
            m.points.push_back(p2);

            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
            m.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(m);
        }

        // 4 propeller discs at arm tips
        for (int i = 0; i < 4; i++) {
            auto m = visualization_msgs::msg::Marker();
            m.header.frame_id = "sensor_head";
            m.header.stamp = stamp;
            m.ns = "drones";
            m.id = base_id + 5 + i;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = px;
            m.pose.position.y = py + arm_offsets[i][0];
            m.pose.position.z = pz + arm_offsets[i][1];
            m.pose.orientation.w = 1.0;
            m.scale.x = prop_size;
            m.scale.y = prop_size;
            m.scale.z = 0.005;  // flat disc
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a * 0.7;
            m.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(m);
        }

        // Label
        {
            auto m = visualization_msgs::msg::Marker();
            m.header.frame_id = "sensor_head";
            m.header.stamp = stamp;
            m.ns = "drones";
            m.id = base_id + 9;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = px;
            m.pose.position.y = py;
            m.pose.position.z = pz + arm_len + 0.05;
            m.pose.orientation.w = 1.0;
            m.scale.z = 0.05;  // text height
            m.text = "T" + std::to_string(target.track_id) +
                     (is_primary ? " [PRI]" : "") +
                     " " + std::to_string(target.range).substr(0, 4) + "m";
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
            m.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(m);
        }
    }

    void deleteDroneMarkers(visualization_msgs::msg::MarkerArray& marker_array, int drone_index)
    {
        int base_id = drone_index * MARKERS_PER_DRONE;
        for (int i = 0; i < MARKERS_PER_DRONE; i++) {
            auto m = visualization_msgs::msg::Marker();
            m.header.frame_id = "sensor_head";
            m.header.stamp = this->now();
            m.ns = "drones";
            m.id = base_id + i;
            m.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(m);
        }
    }

    void trackCallback(const turret_msgs::msg::TrackedTargetArray::SharedPtr msg)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        // Track which drone slots are used this frame
        std::set<int> used_slots;

        int drone_index = 0;
        for (const auto& target : msg->targets) {
            // Only display CONFIRMED or COASTING tracks — skip TENTATIVE noise
            if (target.state != turret_msgs::msg::TrackedTarget::STATE_CONFIRMED &&
                target.state != turret_msgs::msg::TrackedTarget::STATE_COASTING) {
                continue;
            }
            addDroneMarkers(marker_array, target, drone_index,
                           target.track_id == msg->primary_target_id);
            used_slots.insert(drone_index);
            drone_index++;
        }

        // Delete markers for slots that are no longer used
        for (int i = drone_index; i < prev_drone_count_; i++) {
            deleteDroneMarkers(marker_array, i);
        }
        prev_drone_count_ = drone_index;

        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<turret_msgs::msg::TrackedTargetArray>::SharedPtr track_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int prev_drone_count_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
