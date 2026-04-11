#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include "Plant_Parameters.hpp"
#include "../components/core_HelperFunctions.hpp"
#include "../components/core_Saturate.hpp"

class Dynamics {
public: 

    using InputVector = Param::Vector7;
    using StateVector = Param::Vector7;
    using RefVector = Param::Vector10;
    using Scalar = Param::Real;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;

    // Constructor 
    Dynamics();
    void update(const InputVector& inputs);
    const Param::Vector17& getStates() const { return states; }
    const Param::Vector17& getStatesDot() const { return states_dot; }

private: 
    // Helper Functions 
    HelperFunctions helpers; 

    // Private Methods 
    void rk4_kinetics(const InputVector& inputs);

    Param::Vector17 kinetics(const Param::Vector17& states, const InputVector& inputs);

    struct forceOutput {
        Vector3 forces;           // External forces (translational disturbances like drag)
        InputVector moments;      // Actuator torques and external torques
    };

    forceOutput forces_and_moments(const InputVector& inputs);

    // SRP helper functions
    Vector3 compute_SRP_force(const Vector3& r_ECI, const Vector4& q, Param::TimeReal current_time);
    Vector3 compute_SRP_torque(const Vector3& F_srp_body, const Vector3& s_b);

    // Gravity-gradient helper function
    Vector3 compute_gravity_gradient_torque(const Vector3& r_ECI, const Vector4& q);

    // Atmospheric drag helper functions
    Scalar compute_atmospheric_density(const Vector3& r_ECI);
    Vector3 compute_drag_force(const Vector3& r_ECI, const Vector3& v_ECI);

    // Wheel internal dynamics helpers
    Vector4 compute_motor_current_dynamics(const Vector4& V_cmd, const Vector4& omega_wheel, const Vector4& i_motor);
    Vector4 compute_wheel_friction(const Vector4& omega_wheel) const;
    Vector4 compute_wheel_ripple(const Vector4& omega_wheel) const;

    // Private Members
    Scalar Ts;
    Scalar mu_E;
    Scalar r_E;
    Param::Matrix3 I;
    Param::Matrix3 I_inv;
    Scalar G;
    Param::Vector17 states;
    RefVector reference;
    Param::Vector17 states_dot;
    Param::Matrix34 S;
    Scalar I_wheel;
    Param::TimeReal current_time;  // Track simulation time for ephemeris

    // Wheel motor internal dynamics state
    Vector4 i_motor;           // Motor phase current [A]
    Vector4 ripple_phase;      // Ripple phase [rad]
};

#endif // DYNAMICS_HPP