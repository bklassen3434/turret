#include "turret_control/pid_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace turret_control
{

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd),
      output_min_(-std::numeric_limits<double>::max()),
      output_max_(std::numeric_limits<double>::max()),
      integrator_min_(-std::numeric_limits<double>::max()),
      integrator_max_(std::numeric_limits<double>::max()),
      integral_(0.0),
      prev_error_(0.0),
      first_update_(true)
{
}

void PIDController::setGains(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(double min, double max)
{
    output_min_ = min;
    output_max_ = max;
}

void PIDController::setIntegratorLimits(double min, double max)
{
    integrator_min_ = min;
    integrator_max_ = max;
}

double PIDController::compute(double setpoint, double measurement, double dt)
{
    if (dt <= 0.0) {
        return 0.0;
    }

    double error = setpoint - measurement;

    // Proportional term
    double p_term = kp_ * error;

    // Integral term with anti-windup
    integral_ += error * dt;
    integral_ = std::clamp(integral_, integrator_min_, integrator_max_);
    double i_term = ki_ * integral_;

    // Derivative term (on error)
    double d_term = 0.0;
    if (!first_update_) {
        double derivative = (error - prev_error_) / dt;
        d_term = kd_ * derivative;
    }
    first_update_ = false;
    prev_error_ = error;

    // Total output
    double output = p_term + i_term + d_term;

    // Clamp output
    output = std::clamp(output, output_min_, output_max_);

    return output;
}

void PIDController::reset()
{
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_update_ = true;
}

}  // namespace turret_control
