#include "turret_fusion/kalman_filter.hpp"
#include <cmath>

namespace turret_fusion
{

KalmanFilter::KalmanFilter()
    : state_(Eigen::Matrix<double, 6, 1>::Zero()),
      covariance_(Eigen::Matrix<double, 6, 6>::Identity() * 100.0),
      process_noise_pos_(0.1),
      process_noise_vel_(1.0),
      initialized_(false)
{
}

void KalmanFilter::initialize(const Eigen::Vector3d& initial_position,
                              const Eigen::Vector3d& initial_velocity)
{
    state_.head<3>() = initial_position;
    state_.tail<3>() = initial_velocity;

    // Initial covariance: higher uncertainty in velocity
    covariance_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1.0;   // 1m position uncertainty
    covariance_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 10.0; // 10m/s velocity uncertainty

    initialized_ = true;
}

void KalmanFilter::setProcessNoise(double position_noise, double velocity_noise)
{
    process_noise_pos_ = position_noise;
    process_noise_vel_ = velocity_noise;
}

Eigen::Matrix<double, 6, 6> KalmanFilter::getTransitionMatrix(double dt) const
{
    // Constant velocity model:
    // x(k+1) = x(k) + vx(k) * dt
    // vx(k+1) = vx(k)
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    return F;
}

Eigen::Matrix<double, 6, 6> KalmanFilter::getProcessNoise(double dt) const
{
    // Discrete white noise acceleration model
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();

    // Position-position block
    Q(0, 0) = dt4 / 4.0 * process_noise_pos_;
    Q(1, 1) = dt4 / 4.0 * process_noise_pos_;
    Q(2, 2) = dt4 / 4.0 * process_noise_pos_;

    // Position-velocity cross terms
    Q(0, 3) = Q(3, 0) = dt3 / 2.0 * process_noise_pos_;
    Q(1, 4) = Q(4, 1) = dt3 / 2.0 * process_noise_pos_;
    Q(2, 5) = Q(5, 2) = dt3 / 2.0 * process_noise_pos_;

    // Velocity-velocity block
    Q(3, 3) = dt2 * process_noise_vel_;
    Q(4, 4) = dt2 * process_noise_vel_;
    Q(5, 5) = dt2 * process_noise_vel_;

    return Q;
}

void KalmanFilter::predict(double dt)
{
    if (!initialized_) return;

    Eigen::Matrix<double, 6, 6> F = getTransitionMatrix(dt);
    Eigen::Matrix<double, 6, 6> Q = getProcessNoise(dt);

    // Predict state
    state_ = F * state_;

    // Predict covariance
    covariance_ = F * covariance_ * F.transpose() + Q;
}

void KalmanFilter::updatePosition(const Eigen::Vector3d& position,
                                  const Eigen::Matrix3d& measurement_noise)
{
    if (!initialized_) {
        initialize(position);
        return;
    }

    // Measurement matrix: H = [I_3x3 | 0_3x3] (we observe position directly)
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    // Innovation (measurement residual)
    Eigen::Vector3d y = position - H * state_;

    // Innovation covariance
    Eigen::Matrix3d S = H * covariance_ * H.transpose() + measurement_noise;

    // Kalman gain
    Eigen::Matrix<double, 6, 3> K = covariance_ * H.transpose() * S.inverse();

    // Update state
    state_ = state_ + K * y;

    // Update covariance (Joseph form for numerical stability)
    Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    covariance_ = I_KH * covariance_ * I_KH.transpose() + K * measurement_noise * K.transpose();
}

void KalmanFilter::updateBearing(double azimuth, double elevation,
                                 const Eigen::Matrix2d& measurement_noise)
{
    if (!initialized_) return;

    // Current state prediction
    Eigen::Vector3d pos = state_.head<3>();
    double x = pos(0), y = pos(1), z = pos(2);
    double range_xy = std::sqrt(x*x + y*y);
    double range = std::sqrt(x*x + y*y + z*z);

    if (range < 0.1) return;  // Avoid singularity

    // Predicted measurement
    double predicted_az = std::atan2(y, x);
    double predicted_el = std::atan2(z, range_xy);

    // Measurement Jacobian (partial derivatives of [az, el] w.r.t. state)
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();

    // d(azimuth)/d(x,y,z,vx,vy,vz)
    double range_xy2 = range_xy * range_xy;
    H(0, 0) = -y / range_xy2;        // d(az)/dx
    H(0, 1) = x / range_xy2;         // d(az)/dy
    H(0, 2) = 0.0;                   // d(az)/dz

    // d(elevation)/d(x,y,z,vx,vy,vz)
    double range2 = range * range;
    H(1, 0) = -x * z / (range2 * range_xy);
    H(1, 1) = -y * z / (range2 * range_xy);
    H(1, 2) = range_xy / range2;

    // Innovation
    Eigen::Vector2d y_innov;
    y_innov(0) = azimuth - predicted_az;
    y_innov(1) = elevation - predicted_el;

    // Wrap azimuth innovation to [-pi, pi]
    while (y_innov(0) > M_PI) y_innov(0) -= 2.0 * M_PI;
    while (y_innov(0) < -M_PI) y_innov(0) += 2.0 * M_PI;

    // Innovation covariance
    Eigen::Matrix2d S = H * covariance_ * H.transpose() + measurement_noise;

    // Kalman gain
    Eigen::Matrix<double, 6, 2> K = covariance_ * H.transpose() * S.inverse();

    // Update state and covariance
    state_ = state_ + K * y_innov;
    Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    covariance_ = I_KH * covariance_ * I_KH.transpose() + K * measurement_noise * K.transpose();
}

void KalmanFilter::updateRangeRate(double range, double range_rate,
                                   const Eigen::Matrix2d& measurement_noise)
{
    if (!initialized_) return;

    Eigen::Vector3d pos = state_.head<3>();
    Eigen::Vector3d vel = state_.tail<3>();

    double x = pos(0), y = pos(1), z = pos(2);
    double vx = vel(0), vy = vel(1), vz = vel(2);
    double predicted_range = pos.norm();

    if (predicted_range < 0.1) return;  // Avoid singularity

    // Predicted range rate (radial velocity = dot(pos, vel) / |pos|)
    double predicted_range_rate = (x*vx + y*vy + z*vz) / predicted_range;

    // Measurement Jacobian
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();

    // d(range)/d(state)
    H(0, 0) = x / predicted_range;
    H(0, 1) = y / predicted_range;
    H(0, 2) = z / predicted_range;

    // d(range_rate)/d(state) - more complex
    double r3 = predicted_range * predicted_range * predicted_range;
    double dot_pv = x*vx + y*vy + z*vz;

    H(1, 0) = vx / predicted_range - x * dot_pv / r3;
    H(1, 1) = vy / predicted_range - y * dot_pv / r3;
    H(1, 2) = vz / predicted_range - z * dot_pv / r3;
    H(1, 3) = x / predicted_range;
    H(1, 4) = y / predicted_range;
    H(1, 5) = z / predicted_range;

    // Innovation
    Eigen::Vector2d y_innov;
    y_innov(0) = range - predicted_range;
    y_innov(1) = range_rate - predicted_range_rate;

    // Innovation covariance
    Eigen::Matrix2d S = H * covariance_ * H.transpose() + measurement_noise;

    // Kalman gain
    Eigen::Matrix<double, 6, 2> K = covariance_ * H.transpose() * S.inverse();

    // Update state and covariance
    state_ = state_ + K * y_innov;
    Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    covariance_ = I_KH * covariance_ * I_KH.transpose() + K * measurement_noise * K.transpose();
}

}  // namespace turret_fusion
