#include "MotorDriver.hpp"

MotorDriver::MotorDriver() {
    // Initialize parameters
    k_p = 1.0;
    k_d = 0.1;
    I_wheel = 1.13e-6;
    M_PI = 3.14159265358979323846; // Define pi if not defined in math library
    b = Vector4::Constant(0.0); // Intercept for duty cycle mapping (to be tuned)
    m = Vector4::Constant(1.0); // Slope for duty cycle mapping
    omega_w_max = Param::Actuators::omega_w_max;
}

MotorDriver::Vector4 MotorDriver::computeMotorCommands(const Vector4& tau_w_cmd, const Vector4& omega_w, const Scalar& dt) {
    // PD control law to convert desired wheel torques to 4 duty cycle commands, based on the experimentally 
    // found relationship between duty cycle and RPM's for our specific reaction wheels. 
    Vector4 omega_target;

    omega_target = omega_w + (tau_w_cmd / I_wheel) * dt;
    // Saturate target wheel speeds to max limits
    omega_target = saturateSymmetric(omega_target, omega_w_max);
    // Convert to RPM
    omega_target = omega_target * (60.0 / (2.0 * M_PI)); // rad/s to RPM
    // Convert using linear mapping 
    Vector4 duty_cycle_base; 
    for (int i = 0; i < 4; ++i) {
        duty_cycle_base(i) = (omega_target(i) - b(i)) / m(i);
    }

    // PD control to refine duty cycle commands 
    Vector4 duty_cycle_pd;
    duty_cycle_pd = duty_cycle_base + k_p * (omega_target - omega_w) + k_d * ((omega_target - omega_w) / dt);
    return duty_cycle_pd;
}