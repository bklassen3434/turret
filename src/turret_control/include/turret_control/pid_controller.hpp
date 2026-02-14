#ifndef TURRET_CONTROL__PID_CONTROLLER_HPP_
#define TURRET_CONTROL__PID_CONTROLLER_HPP_

namespace turret_control
{

/**
 * @brief Simple PID controller with anti-windup
 */
class PIDController
{
public:
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0);

    /**
     * @brief Set PID gains
     */
    void setGains(double kp, double ki, double kd);

    /**
     * @brief Set output limits for anti-windup
     */
    void setOutputLimits(double min, double max);

    /**
     * @brief Set integrator limits
     */
    void setIntegratorLimits(double min, double max);

    /**
     * @brief Compute control output
     * @param setpoint Desired value
     * @param measurement Current value
     * @param dt Time step (seconds)
     * @return Control output
     */
    double compute(double setpoint, double measurement, double dt);

    /**
     * @brief Reset integrator and derivative state
     */
    void reset();

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double integrator_min_, integrator_max_;

    double integral_;
    double prev_error_;
    bool first_update_;
};

}  // namespace turret_control

#endif  // TURRET_CONTROL__PID_CONTROLLER_HPP_
