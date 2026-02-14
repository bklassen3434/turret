#ifndef TURRET_FUSION__KALMAN_FILTER_HPP_
#define TURRET_FUSION__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace turret_fusion
{

/**
 * @brief Linear Kalman Filter for 3D target tracking
 *
 * State vector: [x, y, z, vx, vy, vz]^T
 * - Position (x, y, z) in meters
 * - Velocity (vx, vy, vz) in m/s
 */
class KalmanFilter
{
public:
    KalmanFilter();

    /**
     * @brief Initialize the filter with an initial state
     * @param initial_position Initial position estimate
     * @param initial_velocity Initial velocity estimate (default zero)
     */
    void initialize(const Eigen::Vector3d& initial_position,
                    const Eigen::Vector3d& initial_velocity = Eigen::Vector3d::Zero());

    /**
     * @brief Predict state forward in time
     * @param dt Time step in seconds
     */
    void predict(double dt);

    /**
     * @brief Update state with a position measurement (e.g., from camera + radar range)
     * @param position Measured position
     * @param measurement_noise Measurement noise covariance (3x3)
     */
    void updatePosition(const Eigen::Vector3d& position,
                        const Eigen::Matrix3d& measurement_noise);

    /**
     * @brief Update state with bearing-only measurement (camera)
     * @param azimuth Measured azimuth angle (radians)
     * @param elevation Measured elevation angle (radians)
     * @param measurement_noise 2x2 covariance for [azimuth, elevation]
     */
    void updateBearing(double azimuth, double elevation,
                       const Eigen::Matrix2d& measurement_noise);

    /**
     * @brief Update state with range and range-rate measurement (radar)
     * @param range Measured range (meters)
     * @param range_rate Measured range rate (m/s)
     * @param measurement_noise 2x2 covariance for [range, range_rate]
     */
    void updateRangeRate(double range, double range_rate,
                         const Eigen::Matrix2d& measurement_noise);

    // Getters
    Eigen::Vector3d getPosition() const { return state_.head<3>(); }
    Eigen::Vector3d getVelocity() const { return state_.tail<3>(); }
    Eigen::Matrix3d getPositionCovariance() const { return covariance_.block<3,3>(0, 0); }
    Eigen::Matrix3d getVelocityCovariance() const { return covariance_.block<3,3>(3, 3); }
    Eigen::Matrix<double, 6, 6> getCovariance() const { return covariance_; }
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Set process noise parameters
     * @param position_noise Position process noise (m^2/s^3)
     * @param velocity_noise Velocity process noise (m^2/s)
     */
    void setProcessNoise(double position_noise, double velocity_noise);

private:
    // State: [x, y, z, vx, vy, vz]
    Eigen::Matrix<double, 6, 1> state_;

    // State covariance (6x6)
    Eigen::Matrix<double, 6, 6> covariance_;

    // Process noise parameters
    double process_noise_pos_;
    double process_noise_vel_;

    bool initialized_;

    /**
     * @brief Build the state transition matrix for constant velocity model
     * @param dt Time step
     */
    Eigen::Matrix<double, 6, 6> getTransitionMatrix(double dt) const;

    /**
     * @brief Build the process noise matrix
     * @param dt Time step
     */
    Eigen::Matrix<double, 6, 6> getProcessNoise(double dt) const;
};

}  // namespace turret_fusion

#endif  // TURRET_FUSION__KALMAN_FILTER_HPP_
