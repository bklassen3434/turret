#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>

#include "turret_fusion/kalman_filter.hpp"
#include "turret_msgs/msg/camera_detection.hpp"
#include "turret_msgs/msg/radar_detection.hpp"
#include "turret_msgs/msg/tracked_target.hpp"
#include "turret_msgs/msg/tracked_target_array.hpp"

struct Track {
    uint32_t id;
    uint8_t state;
    turret_fusion::KalmanFilter kf;
    rclcpp::Time last_camera_time;
    rclcpp::Time last_radar_time;
    rclcpp::Time last_update_time;
    std::string classification;
    bool camera_contributing;
    bool radar_contributing;
    int hit_count;
};

class FusionNode : public rclcpp::Node
{
public:
    FusionNode() : Node("fusion_node")
    {
        // Declare parameters
        this->declare_parameter("update_rate", 50.0);
        this->declare_parameter("camera_azimuth_noise", 0.01);
        this->declare_parameter("camera_elevation_noise", 0.01);
        this->declare_parameter("radar_range_noise", 0.1);
        this->declare_parameter("radar_rate_noise", 0.05);
        this->declare_parameter("track_timeout", 2.0);
        this->declare_parameter("coast_timeout", 0.5);
        this->declare_parameter("initial_range_estimate", 5.0);
        this->declare_parameter("association_threshold_camera", 0.15);  // radians
        this->declare_parameter("association_threshold_radar", 1.0);    // meters
        this->declare_parameter("min_hits", 3);

        // Get parameters
        double update_rate = this->get_parameter("update_rate").as_double();
        camera_az_noise_ = this->get_parameter("camera_azimuth_noise").as_double();
        camera_el_noise_ = this->get_parameter("camera_elevation_noise").as_double();
        radar_range_noise_ = this->get_parameter("radar_range_noise").as_double();
        radar_rate_noise_ = this->get_parameter("radar_rate_noise").as_double();
        track_timeout_ = this->get_parameter("track_timeout").as_double();
        coast_timeout_ = this->get_parameter("coast_timeout").as_double();
        initial_range_ = this->get_parameter("initial_range_estimate").as_double();
        assoc_thresh_camera_ = this->get_parameter("association_threshold_camera").as_double();
        assoc_thresh_radar_ = this->get_parameter("association_threshold_radar").as_double();
        min_hits_ = this->get_parameter("min_hits").as_int();

        // Subscribers
        camera_sub_ = this->create_subscription<turret_msgs::msg::CameraDetection>(
            "camera/detections", 10,
            std::bind(&FusionNode::cameraCallback, this, std::placeholders::_1));

        radar_sub_ = this->create_subscription<turret_msgs::msg::RadarDetection>(
            "radar/detections", 10,
            std::bind(&FusionNode::radarCallback, this, std::placeholders::_1));

        // Publisher
        track_pub_ = this->create_publisher<turret_msgs::msg::TrackedTargetArray>(
            "tracks", 10);

        // Timer for publishing tracks
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate),
            std::bind(&FusionNode::publishTracks, this));

        last_publish_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "Multi-target fusion node initialized (assoc_cam=%.2f rad, assoc_radar=%.1f m, min_hits=%d)",
            assoc_thresh_camera_, assoc_thresh_radar_, min_hits_);
    }

private:
    // Predict a track's KF to the given time
    void predictTrackTo(Track& track, rclcpp::Time now)
    {
        double dt = (now - track.last_update_time).seconds();
        if (dt > 0.001) {
            track.kf.predict(dt);
            track.last_update_time = now;
        }
    }

    // Get azimuth/elevation from a track's current state
    void getTrackBearing(const Track& track, double& az, double& el) const
    {
        Eigen::Vector3d pos = track.kf.getPosition();
        az = std::atan2(pos(1), pos(0));
        el = std::atan2(pos(2), std::sqrt(pos(0)*pos(0) + pos(1)*pos(1)));
    }

    double getTrackRange(const Track& track) const
    {
        return track.kf.getPosition().norm();
    }

    // Find the best matching track for a camera detection
    // Returns track ID, or 0 if no match
    uint32_t associateCamera(double az, double el) const
    {
        uint32_t best_id = 0;
        double best_dist = assoc_thresh_camera_;

        for (const auto& [id, track] : tracks_) {
            if (track.state == turret_msgs::msg::TrackedTarget::STATE_LOST) continue;

            double track_az, track_el;
            getTrackBearing(track, track_az, track_el);

            double dist = std::sqrt((az - track_az)*(az - track_az) +
                                    (el - track_el)*(el - track_el));
            if (dist < best_dist) {
                best_dist = dist;
                best_id = id;
            }
        }
        return best_id;
    }

    // Find the best matching track for a radar detection
    uint32_t associateRadar(double range, double az, double el) const
    {
        uint32_t best_id = 0;
        double best_dist = assoc_thresh_radar_;

        for (const auto& [id, track] : tracks_) {
            if (track.state == turret_msgs::msg::TrackedTarget::STATE_LOST) continue;

            double track_az, track_el;
            getTrackBearing(track, track_az, track_el);
            double track_range = getTrackRange(track);

            // Combined range + angle distance
            double range_diff = range - track_range;
            double az_diff = az - track_az;
            double el_diff = el - track_el;
            double dist = std::sqrt(range_diff*range_diff + 4.0*(az_diff*az_diff + el_diff*el_diff));

            if (dist < best_dist) {
                best_dist = dist;
                best_id = id;
            }
        }
        return best_id;
    }

    Track& createTrack(const Eigen::Vector3d& position, rclcpp::Time now)
    {
        uint32_t id = next_track_id_++;
        Track track;
        track.id = id;
        track.state = turret_msgs::msg::TrackedTarget::STATE_TENTATIVE;
        track.kf.setProcessNoise(0.5, 2.0);
        track.kf.initialize(position);
        track.last_camera_time = now;
        track.last_radar_time = now;
        track.last_update_time = now;
        track.camera_contributing = false;
        track.radar_contributing = false;
        track.hit_count = 1;

        tracks_[id] = std::move(track);
        RCLCPP_INFO(this->get_logger(), "New track %u created", id);
        return tracks_[id];
    }

    void promoteTrack(Track& track)
    {
        track.hit_count++;
        if (track.state == turret_msgs::msg::TrackedTarget::STATE_TENTATIVE &&
            track.hit_count >= min_hits_) {
            track.state = turret_msgs::msg::TrackedTarget::STATE_CONFIRMED;
            RCLCPP_INFO(this->get_logger(), "Track %u confirmed", track.id);
        } else if (track.state == turret_msgs::msg::TrackedTarget::STATE_COASTING) {
            track.state = turret_msgs::msg::TrackedTarget::STATE_CONFIRMED;
        }
    }

    void cameraCallback(const turret_msgs::msg::CameraDetection::SharedPtr msg)
    {
        rclcpp::Time now = this->now();

        // Predict all tracks to current time
        for (auto& [id, track] : tracks_) {
            predictTrackTo(track, now);
        }

        uint32_t match_id = associateCamera(msg->azimuth, msg->elevation);

        if (match_id != 0) {
            // Update existing track
            Track& track = tracks_[match_id];
            track.last_camera_time = now;
            track.camera_contributing = true;
            track.classification = msg->classification;

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0, 0) = camera_az_noise_ * camera_az_noise_;
            R(1, 1) = camera_el_noise_ * camera_el_noise_;
            track.kf.updateBearing(msg->azimuth, msg->elevation, R);

            promoteTrack(track);
        } else {
            // Create new tentative track from camera bearing + assumed range
            double range = initial_range_;
            double x = range * std::cos(msg->elevation) * std::cos(msg->azimuth);
            double y = range * std::cos(msg->elevation) * std::sin(msg->azimuth);
            double z = range * std::sin(msg->elevation);

            Track& track = createTrack(Eigen::Vector3d(x, y, z), now);
            track.camera_contributing = true;
            track.classification = msg->classification;
        }
    }

    void radarCallback(const turret_msgs::msg::RadarDetection::SharedPtr msg)
    {
        rclcpp::Time now = this->now();

        // Predict all tracks to current time
        for (auto& [id, track] : tracks_) {
            predictTrackTo(track, now);
        }

        uint32_t match_id = associateRadar(msg->range, msg->azimuth, msg->elevation);

        if (match_id != 0) {
            // Update existing track
            Track& track = tracks_[match_id];
            track.last_radar_time = now;
            track.radar_contributing = true;

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0, 0) = radar_range_noise_ * radar_range_noise_;
            R(1, 1) = radar_rate_noise_ * radar_rate_noise_;
            track.kf.updateRangeRate(msg->range, msg->range_rate, R);

            promoteTrack(track);
        } else {
            // Create new tentative track from radar
            double x = msg->range * std::cos(msg->elevation) * std::cos(msg->azimuth);
            double y = msg->range * std::cos(msg->elevation) * std::sin(msg->azimuth);
            double z = msg->range * std::sin(msg->elevation);

            Track& track = createTrack(Eigen::Vector3d(x, y, z), now);
            track.radar_contributing = true;
        }
    }

    void publishTracks()
    {
        rclcpp::Time now = this->now();

        // Predict all tracks and update states
        std::vector<uint32_t> to_remove;
        for (auto& [id, track] : tracks_) {
            predictTrackTo(track, now);

            double camera_age = (now - track.last_camera_time).seconds();
            double radar_age = (now - track.last_radar_time).seconds();
            double time_since_update = std::min(camera_age, radar_age);

            track.camera_contributing = (camera_age < coast_timeout_);
            track.radar_contributing = (radar_age < coast_timeout_);

            // Update track lifecycle state
            if (!track.camera_contributing && !track.radar_contributing) {
                if (time_since_update > track_timeout_) {
                    track.state = turret_msgs::msg::TrackedTarget::STATE_LOST;
                    to_remove.push_back(id);
                } else if (track.state == turret_msgs::msg::TrackedTarget::STATE_CONFIRMED) {
                    track.state = turret_msgs::msg::TrackedTarget::STATE_COASTING;
                } else if (track.state == turret_msgs::msg::TrackedTarget::STATE_TENTATIVE &&
                           time_since_update > coast_timeout_) {
                    // Tentative tracks that never got confirmed — drop faster
                    to_remove.push_back(id);
                }
            }
        }

        // Remove LOST tracks
        for (uint32_t id : to_remove) {
            RCLCPP_INFO(this->get_logger(), "Track %u removed", id);
            tracks_.erase(id);
        }

        // Build track array message
        auto track_array = turret_msgs::msg::TrackedTargetArray();
        track_array.header.stamp = now;
        track_array.header.frame_id = "radar_link";

        // Find primary target (closest CONFIRMED track)
        uint32_t primary_id = 0;
        double min_range = std::numeric_limits<double>::max();

        for (const auto& [id, track] : tracks_) {
            auto target = turret_msgs::msg::TrackedTarget();
            target.header = track_array.header;
            target.track_id = track.id;
            target.state = track.state;

            Eigen::Vector3d pos = track.kf.getPosition();
            Eigen::Vector3d vel = track.kf.getVelocity();
            Eigen::Matrix3d pos_cov = track.kf.getPositionCovariance();
            Eigen::Matrix3d vel_cov = track.kf.getVelocityCovariance();

            target.position.x = pos(0);
            target.position.y = pos(1);
            target.position.z = pos(2);

            target.velocity.x = vel(0);
            target.velocity.y = vel(1);
            target.velocity.z = vel(2);

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    target.position_covariance[i * 3 + j] = pos_cov(i, j);
                    target.velocity_covariance[i * 3 + j] = vel_cov(i, j);
                }
            }

            double range = pos.norm();
            target.range = range;
            target.azimuth = std::atan2(pos(1), pos(0));
            target.elevation = std::atan2(pos(2), std::sqrt(pos(0)*pos(0) + pos(1)*pos(1)));

            target.camera_contributing = track.camera_contributing;
            target.radar_contributing = track.radar_contributing;
            target.classification = track.classification;

            double camera_age = (now - track.last_camera_time).seconds();
            double radar_age = (now - track.last_radar_time).seconds();
            target.time_since_update = std::min(camera_age, radar_age);

            // Confidence based on covariance and sensor status
            double pos_uncertainty = std::sqrt(pos_cov.trace());
            target.confidence = std::max(0.0, 1.0 - pos_uncertainty / 10.0);
            if (!track.camera_contributing) target.confidence *= 0.8;
            if (!track.radar_contributing) target.confidence *= 0.8;

            track_array.targets.push_back(target);

            // Primary target = closest CONFIRMED track
            if (track.state == turret_msgs::msg::TrackedTarget::STATE_CONFIRMED &&
                range < min_range) {
                min_range = range;
                primary_id = track.id;
            }
        }

        track_array.primary_target_id = primary_id;
        track_pub_->publish(track_array);
    }

    // Tracks
    std::map<uint32_t, Track> tracks_;
    uint32_t next_track_id_ = 1;

    // Subscribers and publisher
    rclcpp::Subscription<turret_msgs::msg::CameraDetection>::SharedPtr camera_sub_;
    rclcpp::Subscription<turret_msgs::msg::RadarDetection>::SharedPtr radar_sub_;
    rclcpp::Publisher<turret_msgs::msg::TrackedTargetArray>::SharedPtr track_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double camera_az_noise_;
    double camera_el_noise_;
    double radar_range_noise_;
    double radar_rate_noise_;
    double track_timeout_;
    double coast_timeout_;
    double initial_range_;
    double assoc_thresh_camera_;
    double assoc_thresh_radar_;
    int min_hits_;

    // State
    rclcpp::Time last_publish_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}
