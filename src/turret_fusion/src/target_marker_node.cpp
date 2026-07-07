/**
 * Target Marker Node
 *
 * Publishes RViz visualization markers for the fused tracks so a human can see,
 * in 3D, exactly what the tracker believes:
 *   - Drone-shaped markers placed at the track's TRUE 3D position (not a fixed
 *     shell), in the fusion frame (radar_link) so RViz renders real geometry.
 *   - A velocity arrow showing the Kalman-estimated heading/speed.
 *   - A translucent covariance ellipsoid showing the filter's position
 *     uncertainty (shrinks as camera + radar agree).
 *   - A fading trajectory trail (stored in the fixed base_link frame) showing
 *     the flight path.
 *   - A text label with track ID, primary flag, range, contributing sensors
 *     and confidence.
 *
 * Colour encodes role/state: red = primary, orange = secondary, dim yellow =
 * coasting (predicting with no live sensor).
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <map>
#include <set>
#include <cstdio>
#include "turret_msgs/msg/tracked_target_array.hpp"

class TargetMarkerNode : public rclcpp::Node
{
public:
    TargetMarkerNode() : Node("target_marker_node")
    {
        fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "base_link");
        trail_length_ = this->declare_parameter<int>("trail_length", 120);
        trail_min_step_ = this->declare_parameter<double>("trail_min_step", 0.02);
        cov_sigma_ = this->declare_parameter<double>("covariance_sigma", 2.0);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        track_sub_ = this->create_subscription<turret_msgs::msg::TrackedTargetArray>(
            "/tracks", 10,
            std::bind(&TargetMarkerNode::trackCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/target_markers", 10);

        RCLCPP_INFO(this->get_logger(), "Target marker node initialized (fixed_frame=%s)",
                    fixed_frame_.c_str());
    }

private:
    // Each drone uses a block of marker IDs: base_id + 0..15
    // 0 = body, 1-4 = arms, 5-8 = propellers, 9 = label,
    // 10 = velocity arrow, 11 = covariance ellipsoid, 12 = trajectory trail
    static constexpr int MARKERS_PER_DRONE = 16;

    struct Rgba { float r, g, b, a; };

    // Colour by role/state so the operator can read the situation at a glance.
    Rgba colourFor(const turret_msgs::msg::TrackedTarget& t, bool is_primary) const
    {
        if (t.state == turret_msgs::msg::TrackedTarget::STATE_COASTING) {
            return {0.9f, 0.8f, 0.1f, 0.85f};   // dim yellow: predicting, no live sensor
        }
        if (is_primary) {
            return {1.0f, 0.0f, 0.0f, 1.0f};    // bright red: primary target
        }
        return {1.0f, 0.5f, 0.0f, 0.85f};       // orange: secondary
    }

    visualization_msgs::msg::Marker baseMarker(const std::string& frame, int id,
                                               rclcpp::Time stamp) const
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = stamp;
        m.ns = "drones";
        m.id = id;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        return m;
    }

    void addDroneMarkers(visualization_msgs::msg::MarkerArray& marker_array,
                         const turret_msgs::msg::TrackedTarget& target,
                         const std::string& track_frame,
                         int drone_index, bool is_primary)
    {
        const int base_id = drone_index * MARKERS_PER_DRONE;
        const auto stamp = this->now();
        const Rgba c = colourFor(target, is_primary);

        // TRUE 3D position in the fusion frame — RViz transforms it for us.
        const double px = target.position.x;
        const double py = target.position.y;
        const double pz = target.position.z;

        const double body_size = 0.08;
        const double arm_len = 0.12;
        const double prop_size = 0.04;

        // Body cube
        {
            auto m = baseMarker(track_frame, base_id + 0, stamp);
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.pose.position.x = px;
            m.pose.position.y = py;
            m.pose.position.z = pz;
            m.scale.x = body_size;
            m.scale.y = body_size * 1.5;
            m.scale.z = body_size * 0.4;
            m.color.r = c.r; m.color.g = c.g; m.color.b = c.b; m.color.a = c.a;
            marker_array.markers.push_back(m);
        }

        // 4 arms as line strips radiating from the body
        const double arm_offsets[4][2] = {
            {-arm_len, -arm_len},  // front-left
            { arm_len, -arm_len},  // front-right
            {-arm_len,  arm_len},  // rear-left
            { arm_len,  arm_len},  // rear-right
        };
        for (int i = 0; i < 4; i++) {
            auto m = baseMarker(track_frame, base_id + 1 + i, stamp);
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.scale.x = 0.01;  // line width
            geometry_msgs::msg::Point p1, p2;
            p1.x = px; p1.y = py; p1.z = pz;
            p2.x = px; p2.y = py + arm_offsets[i][0]; p2.z = pz + arm_offsets[i][1];
            m.points.push_back(p1);
            m.points.push_back(p2);
            m.color.r = c.r; m.color.g = c.g; m.color.b = c.b; m.color.a = c.a;
            marker_array.markers.push_back(m);
        }

        // 4 propeller discs at arm tips
        for (int i = 0; i < 4; i++) {
            auto m = baseMarker(track_frame, base_id + 5 + i, stamp);
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.pose.position.x = px;
            m.pose.position.y = py + arm_offsets[i][0];
            m.pose.position.z = pz + arm_offsets[i][1];
            m.scale.x = prop_size;
            m.scale.y = prop_size;
            m.scale.z = 0.005;  // flat disc
            m.color.r = c.r; m.color.g = c.g; m.color.b = c.b; m.color.a = c.a * 0.7f;
            marker_array.markers.push_back(m);
        }

        // Velocity arrow (Kalman-estimated heading/speed), only if moving
        const double speed = std::sqrt(
            target.velocity.x * target.velocity.x +
            target.velocity.y * target.velocity.y +
            target.velocity.z * target.velocity.z);
        if (speed > 0.02) {
            auto m = baseMarker(track_frame, base_id + 10, stamp);
            m.type = visualization_msgs::msg::Marker::ARROW;
            const double lookahead = 2.0;  // seconds of predicted travel, for visibility
            geometry_msgs::msg::Point tail, head;
            tail.x = px; tail.y = py; tail.z = pz;
            head.x = px + target.velocity.x * lookahead;
            head.y = py + target.velocity.y * lookahead;
            head.z = pz + target.velocity.z * lookahead;
            m.points.push_back(tail);
            m.points.push_back(head);
            m.scale.x = 0.015;  // shaft diameter
            m.scale.y = 0.04;   // head diameter
            m.scale.z = 0.06;   // head length
            m.color.r = 0.2f; m.color.g = 0.8f; m.color.b = 1.0f; m.color.a = 0.9f;
            marker_array.markers.push_back(m);
        }

        // Covariance ellipsoid (position uncertainty)
        addCovarianceEllipsoid(marker_array, target, track_frame, base_id + 11, stamp, c);

        // Label
        {
            auto m = baseMarker(track_frame, base_id + 9, stamp);
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.pose.position.x = px;
            m.pose.position.y = py;
            m.pose.position.z = pz + arm_len + 0.05;
            m.scale.z = 0.05;  // text height
            char buf[128];
            std::snprintf(buf, sizeof(buf), "T%u%s  %.1fm  [%s%s] %.0f%%",
                          target.track_id,
                          is_primary ? " [PRI]" : "",
                          target.range,
                          target.camera_contributing ? "C" : "-",
                          target.radar_contributing ? "R" : "-",
                          target.confidence * 100.0);
            m.text = buf;
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
            marker_array.markers.push_back(m);
        }
    }

    // Draw the position covariance as a translucent ellipsoid oriented by its
    // principal axes. Uncertainty shrinks visibly as the filter converges.
    void addCovarianceEllipsoid(visualization_msgs::msg::MarkerArray& marker_array,
                                const turret_msgs::msg::TrackedTarget& target,
                                const std::string& track_frame, int id,
                                rclcpp::Time stamp, const Rgba& c)
    {
        Eigen::Matrix3d cov;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cov(i, j) = target.position_covariance[i * 3 + j];

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        if (solver.info() != Eigen::Success) return;

        Eigen::Vector3d evals = solver.eigenvalues();
        Eigen::Matrix3d evecs = solver.eigenvectors();

        // Ensure a proper (right-handed) rotation so the quaternion is valid.
        if (evecs.determinant() < 0) evecs.col(2) = -evecs.col(2);
        Eigen::Quaterniond q(evecs);
        q.normalize();

        auto m = baseMarker(track_frame, id, stamp);
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position.x = target.position.x;
        m.pose.position.y = target.position.y;
        m.pose.position.z = target.position.z;
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();

        // Diameter along each axis = 2 * cov_sigma * standard deviation, clamped
        // so a freshly-initialised (huge covariance) track doesn't fill the view.
        for (int i = 0; i < 3; i++) {
            double sigma = std::sqrt(std::max(evals(i), 1e-6));
            double diameter = 2.0 * cov_sigma_ * sigma;
            diameter = std::min(std::max(diameter, 0.03), 2.0);
            if (i == 0) m.scale.x = diameter;
            else if (i == 1) m.scale.y = diameter;
            else m.scale.z = diameter;
        }
        m.color.r = c.r; m.color.g = c.g; m.color.b = c.b; m.color.a = 0.20f;
        marker_array.markers.push_back(m);
    }

    // Append the current position (transformed into the fixed frame) to a track's
    // trail and emit it as a fading line strip. Fixed-frame storage keeps the
    // trail world-stable even as the turret (and thus radar_link) rotates.
    void addTrail(visualization_msgs::msg::MarkerArray& marker_array,
                  const turret_msgs::msg::TrackedTarget& target,
                  const std::string& track_frame, int drone_index, const Rgba& c)
    {
        geometry_msgs::msg::PointStamped in, out;
        in.header.frame_id = track_frame;
        in.header.stamp = rclcpp::Time(0);  // latest available transform
        in.point = target.position;
        try {
            // Zero timeout: use the latest available transform without blocking the
            // executor (this runs in the single-threaded subscription callback).
            out = tf_buffer_->transform(in, fixed_frame_, tf2::durationFromSec(0.0));
        } catch (const tf2::TransformException& ex) {
            return;  // no TF yet — skip the trail this frame, drone markers still show
        }

        auto& trail = trails_[target.track_id];
        if (trail.empty() ||
            std::hypot(std::hypot(out.point.x - trail.back().x,
                                  out.point.y - trail.back().y),
                       out.point.z - trail.back().z) > trail_min_step_) {
            trail.push_back(out.point);
            while (static_cast<int>(trail.size()) > trail_length_) trail.pop_front();
        }
        if (trail.size() < 2) return;

        const int base_id = drone_index * MARKERS_PER_DRONE;
        auto m = baseMarker(fixed_frame_, base_id + 12, this->now());
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.015;  // line width
        m.color.r = c.r; m.color.g = c.g; m.color.b = c.b; m.color.a = 0.6f;
        for (const auto& p : trail) m.points.push_back(p);
        marker_array.markers.push_back(m);
    }

    void deleteDroneMarkers(visualization_msgs::msg::MarkerArray& marker_array, int drone_index)
    {
        const int base_id = drone_index * MARKERS_PER_DRONE;
        for (int i = 0; i < MARKERS_PER_DRONE; i++) {
            auto m = baseMarker(fixed_frame_, base_id + i, this->now());
            m.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(m);
        }
    }

    void trackCallback(const turret_msgs::msg::TrackedTargetArray::SharedPtr msg)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        const std::string track_frame =
            msg->header.frame_id.empty() ? "radar_link" : msg->header.frame_id;

        std::set<uint32_t> live_ids;
        int drone_index = 0;
        for (const auto& target : msg->targets) {
            live_ids.insert(target.track_id);
            // Only display CONFIRMED or COASTING tracks — skip TENTATIVE noise
            if (target.state != turret_msgs::msg::TrackedTarget::STATE_CONFIRMED &&
                target.state != turret_msgs::msg::TrackedTarget::STATE_COASTING) {
                continue;
            }
            const bool is_primary = (target.track_id == msg->primary_target_id);
            const Rgba c = colourFor(target, is_primary);
            addDroneMarkers(marker_array, target, track_frame, drone_index, is_primary);
            addTrail(marker_array, target, track_frame, drone_index, c);
            drone_index++;
        }

        // Delete markers for slots that are no longer used
        for (int i = drone_index; i < prev_drone_count_; i++) {
            deleteDroneMarkers(marker_array, i);
        }
        prev_drone_count_ = drone_index;

        // Drop trail history for tracks that have disappeared entirely
        for (auto it = trails_.begin(); it != trails_.end(); ) {
            if (live_ids.count(it->first) == 0) it = trails_.erase(it);
            else ++it;
        }

        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<turret_msgs::msg::TrackedTargetArray>::SharedPtr track_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::map<uint32_t, std::deque<geometry_msgs::msg::Point>> trails_;
    std::string fixed_frame_;
    int trail_length_;
    double trail_min_step_;
    double cov_sigma_;
    int prev_drone_count_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
