#ifndef CORE_MOTORDRIVER_HPP
#define CORE_MOTORDRIVER_HPP

#include "core_Parameters.hpp"
#include "core_Saturate.hpp"

class MotorDriver {
public: 

    // Constructor 
    MotorDriver();
    using Vector4 = Param::Vector4;
    using Scalar = Param::Real;
    Vector4 computeMotorCommands(const Vector4& tau_w_cmd, const Vector4& omega_w, const Scalar& dt);

private:
    // Private Members
    Scalar k_p, k_d; // PD control gains 
    Scalar I_wheel; // Wheel inertia
    Scalar M_PI; // Pi constant
    Vector4 b; // Intercept for duty cycle mapping
    Vector4 m; // Slope for duty cycle mapping
    Scalar omega_w_max; // Maximum wheel speed (RPM)
};

#endif // CORE_MOTORDRIVER_HPP