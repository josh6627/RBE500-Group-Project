#include "controller_cpp/PID.hpp"
#include <cmath>

/**
 * @brief Construct a new PID::PID object
 *
 */
PID::PID() {
    reset_variables();
    set_constants(0, 0, 0, 0, 0);
}

/**
 * @brief Construct a new PID::PID object
 *
 * @param kp
 * @param ki
 * @param kd
 * @param kf
 * @param start_i
 * @param name
 */
PID::PID(double kp, double ki, double kd, double kf, double start_i, std::string name) {
    reset_variables();
    set_name(name);
    set_constants(kp, ki, kd, kf, start_i);
}

/**
 * @brief Reset class variables
 *
 */
void PID::reset_variables() {
    output = 0;
    target = 0;
    error = 0;
    integral = 0;
    derivative = 0;
    prev_error = 0;
}

/**
 * @brief Set PID constants
 *
 * @param p
 * @param i
 * @param d
 * @param f
 * @param start_i
 */
void PID::set_constants(double kp, double ki, double kd, double kf, double start_i) {
    constants.kp = kp;
    constants.ki = ki;
    constants.kd = kd;
    constants.kf = kf;
    constants.start_i = start_i;
}

/**
 * @brief Set PID target
 *
 * @param target
 */
void PID::set_target(double target) {
    reset_variables();
    this->target = target;
}

/**
 * @brief Return target
 *
 */
double PID::return_target() { return target; }

/**
 * @brief Return PID constants
 *
 * @return PID::Constants
 */
PID::Constants PID::return_constants() { return constants; }

/**
 * @brief set PID object name
 *
 * @param name
 */
void PID::set_name(std::string name) { this->name = name; }

/**
 * @brief Compute PID
 *
 * @param input
 * @return double
 */
double PID::compute(double input, double dt) {
    error = target - input;

    if (dt > 1.0e-6)
        derivative = (error - prev_error) / dt;
    else
        derivative = 0.0;

    if (constants.ki != 0.0) {
        if (std::fabs(error) < constants.start_i)
            integral += error * dt;

        if (error * prev_error < 0.0)
            integral = 0.0;
    }

    output = (error * constants.kp) + (integral * constants.ki) + (derivative * constants.kd) +
             (target * constants.kf);

    prev_error = error;
    return output;
}