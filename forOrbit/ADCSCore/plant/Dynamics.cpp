#include "Dynamics.hpp" 
#include <iostream> // For debugging purposes
#include <cmath>
#include "two_body.hpp"
// Constructor 
Dynamics::Dynamics()
    : Ts(PlantParam::SimTime::Ts),
      mu_E(PlantParam::Earth::mu_E),
      r_E(PlantParam::Earth::r_E),
      I(PlantParam::Spacecraft::I),
      I_inv(Math::inverse3x3(PlantParam::Spacecraft::I)),
      G(PlantParam::Earth::G),
      states(PlantParam::Vector17::Zero()),
      reference(PlantParam::Vector10::Zero()),
      states_dot(PlantParam::Vector17::Zero()),
      S(PlantParam::Actuators::S),
      I_wheel(PlantParam::Actuators::I_wheel),
      current_time(PlantParam::SimTime::epoch_timestamp),
      i_motor(PlantParam::Vector4::Zero()),
      ripple_phase(PlantParam::Vector4::Zero())
{
    states.setSegment(0, PlantParam::Orbit::InitialState.r_ECI);
    states.setSegment(3, PlantParam::Orbit::InitialState.v_ECI);
    states.setSegment(6, PlantParam::Orbit::InitialState.q0);
    states.setSegment(10, PlantParam::Orbit::InitialState.omega);
    states.setSegment(13, PlantParam::Orbit::InitialState.omega_wheel);
}

void Dynamics::update(const InputVector& inputs) {
    // Update motor current dynamics (electrical model)
    if (PlantParam::Actuators::WheelDynamics::enable_motor_dynamics) {
        Vector4 tau_cmd = inputs.segment<4>(0);
        Vector4 omega_wheel = states.segment<4>(13);
        
        Scalar Kt = PlantParam::Actuators::WheelMotor::Kt;
        Scalar Ke = PlantParam::Actuators::WheelMotor::Ke;
        Scalar R = PlantParam::Actuators::WheelMotor::R;
        
        // Motor electrical time constant tau_e = L/R = 0.12mH / 3.95 ohm = 30 us
        // Since tau_e << Ts (sample period), electrical dynamics reach steady-state
        // nearly instantaneously relative to our sample rate. Use quasi-static model:
        // 
        // Steady-state current: i_ss = (V_cmd - Ke * omega) / R
        // With voltage command: V_cmd = R * i_desired + Ke * omega (back-EMF compensation)
        // We get: i_ss = i_desired = tau_cmd / Kt
        //
        // However, we model a first-order motor lag (mechanical time constant tau_m)
        // to capture the motor's mechanical response time (inertia + controller lag)
        Scalar tau_m = PlantParam::Actuators::WheelMotor::tau_m;
        
        // First-order lag on current: tau_m * di/dt + i = i_desired
        // Discrete update: i[k+1] = i[k] + (Ts/tau_m) * (i_desired - i[k])
        Scalar alpha = Ts / tau_m;
        if (alpha > static_cast<Scalar>(1.0)) {
            alpha = static_cast<Scalar>(1.0);  // Ensure stability
        }
        
        for (int i = 0; i < 4; ++i) {
            Scalar i_desired = tau_cmd(i) / Kt;
            i_motor(i) += alpha * (i_desired - i_motor(i));
        }
        
        // Clamp current to realistic limits (based on max torque)
        Scalar i_max = PlantParam::Actuators::tau_w_max / Kt;
        for (int i = 0; i < 4; ++i) {
            if (i_motor(i) > i_max) i_motor(i) = i_max;
            if (i_motor(i) < -i_max) i_motor(i) = -i_max;
        }
    }

    // Update ripple phase based on wheel spin rate
    if (PlantParam::Actuators::WheelDynamics::enable_ripple) {
        Vector4 omega_wheel = states.segment<4>(13);
        for (int i = 0; i < 4; ++i) {
            Scalar rate = PlantParam::Actuators::WheelDynamics::ripple_harmonic * std::abs(omega_wheel(i));
            ripple_phase(i) += rate * Ts;
            // Keep phase bounded
            if (ripple_phase(i) > static_cast<Scalar>(2.0) * PlantParam::PI) {
                ripple_phase(i) = std::fmod(ripple_phase(i), static_cast<Scalar>(2.0) * PlantParam::PI);
            }
        }
    }

    rk4_kinetics(inputs);
    current_time += Ts;  // Increment simulation time
}

void Dynamics::rk4_kinetics(const InputVector& inputs) {
    // RK4 Integration 
    PlantParam::Vector17 k1 = kinetics(states, inputs);
    PlantParam::Vector17 k2 = kinetics(states + (Ts / static_cast<Scalar>(2.0)) * k1, inputs);
    PlantParam::Vector17 k3 = kinetics(states + (Ts / static_cast<Scalar>(2.0)) * k2, inputs);
    PlantParam::Vector17 k4 = kinetics(states + Ts * k3, inputs);

    states = states + (Ts / static_cast<Scalar>(6.0)) * (k1 + static_cast<Scalar>(2.0) * k2 + static_cast<Scalar>(2.0) * k3 + k4);
    PlantParam::Quat q_norm;
    q_norm(0) = states(6);
    q_norm.setSegment(1, states.segment<3>(7));
    q_norm.normalize();
    states.setSegment(6, q_norm);
    states.setSegment(13, saturateSymmetric<4>(states.segment<4>(13), PlantParam::Actuators::omega_w_max));
}

PlantParam::Vector17 Dynamics::kinetics(const PlantParam::Vector17& states, const InputVector& inputs) {
    // Unpack States 
    Vector3 r_ECI = states.segment<3>(0);
    Vector3 v_ECI = states.segment<3>(3);
    Quat q = states.segment<4>(6);
    Vector3 omega = states.segment<3>(10);
    Vector4 omega_wheel = states.segment<4>(13);

    // Calculate forces and moments 
    forceOutput fm = forces_and_moments(inputs);
    Vector4 tau_rw = fm.moments.segment<4>(0);
    Vector3 tau_mtq_plus_dist = fm.moments.segment<3>(4);  // MTQ + disturbance torques
    Vector3 F_ext = fm.forces;  // External translational forces (drag)

    Vector3 h_w = S * (I_wheel * omega_wheel); // Reaction Wheel Angular Momentum
    Vector3 tau_ext = tau_mtq_plus_dist - (S * tau_rw);

    Vector3 R_dot = v_ECI;

    Vector3 V_dot = TwoBody::two_body(mu_E, r_E, r_ECI);
    // Add external translational forces (drag, etc.) - divided by mass for acceleration
    V_dot += F_ext / PlantParam::Spacecraft::mass;

    Vector3 q_vec = q.segment<3>(1);
    Scalar q0 = q(0);
    Vector3 q_dot_vec = static_cast<Scalar>(0.5)*(q0*omega + q_vec.cross(omega));
    Scalar q0_dot = static_cast<Scalar>(-0.5) * q_vec.dot(omega);

    Quat q_dot; 
    q_dot(0) = q0_dot;
    q_dot.setSegment(1, q_dot_vec);
    Vector3 omega_dot = I_inv * (tau_ext - omega.cross(I * omega + h_w));

    Vector4 omega_wheel_dot = tau_rw / I_wheel;

    states_dot.setSegment(0, R_dot);
    states_dot.setSegment(3, V_dot);
    states_dot.setSegment(6, q_dot);
    states_dot.setSegment(10, omega_dot);
    states_dot.setSegment(13, omega_wheel_dot);
    return states_dot;
}

Dynamics::forceOutput Dynamics::forces_and_moments(const InputVector& inputs) {
    // Unpack Inputs (reaction wheel torques and magnetorquer dipoles)
    Vector3 tau_mtq = inputs.segment<3>(4);

    Vector3 r_ECI = states.segment<3>(0);
    Vector3 v_ECI = states.segment<3>(3);
    Vector4 q = states.segment<4>(6);
    Vector4 omega_wheel = states.segment<4>(13);

    // Compute external translational forces (atmospheric drag, etc.)
    Vector3 F_ext = Vector3::Zero();

    if (PlantParam::Disturbances::enable_drag) {
        Vector3 F_drag = compute_drag_force(r_ECI, v_ECI);
        F_ext += F_drag;
    }

    // Wheel internal dynamics: friction and torque ripple
    Vector4 tau_fric = Vector4::Zero();
    Vector4 tau_ripple = Vector4::Zero();

    if (PlantParam::Actuators::WheelDynamics::enable_friction) {
        tau_fric = compute_wheel_friction(omega_wheel);
    }

    if (PlantParam::Actuators::WheelDynamics::enable_ripple) {
        tau_ripple = compute_wheel_ripple(omega_wheel);
    }

    // Compute motor torque from current: tau_motor = Kt * i
    Vector4 tau_motor = Vector4::Zero();
    if (PlantParam::Actuators::WheelDynamics::enable_motor_dynamics) {
        Scalar Kt = PlantParam::Actuators::WheelMotor::Kt;
        tau_motor = Kt * i_motor;
    } else {
        // If motor dynamics disabled, use commanded torque directly
        tau_motor = inputs.segment<4>(0);
        tau_motor = saturateSymmetric<4>(tau_motor, PlantParam::Actuators::tau_w_max);
    }

    // Net wheel torque applied to wheel (motor + ripple - friction)
    Vector4 tau_wheel_net = tau_motor + tau_ripple - tau_fric;

    // Compute external disturbance torques (SRP, gravity gradient, etc.)
    Vector3 tau_disturbances = Vector3::Zero();

    if (PlantParam::Disturbances::enable_SRP) {
        Vector3 F_srp_body = compute_SRP_force(r_ECI, q, current_time);

        // Transform to body frame to compute body-frame sun vector
        Vector3 s_i = helpers.earth2sun(helpers.julianDate(current_time)).normalized();
        Vector3 s_b = helpers.quatRotate(helpers.quatconj(q), s_i);

        Vector3 tau_srp = compute_SRP_torque(F_srp_body, s_b);
        tau_disturbances += tau_srp;
    }

    if (PlantParam::Disturbances::enable_gravity_gradient) {
        Vector3 tau_gg = compute_gravity_gradient_torque(r_ECI, q);
        tau_disturbances += tau_gg;
    }

    // Package output: forces field for translational disturbances
    // Moments field packs wheel net torques + disturbance torques + MTQ
    PlantParam::Vector7 moments;
    moments.setSegment(0, tau_wheel_net);
    moments.setSegment(4, tau_mtq + tau_disturbances);

    return forceOutput{F_ext, moments};
}

Dynamics::Vector3 Dynamics::compute_SRP_force(const Vector3& r_ECI, const Vector4& q, Param::TimeReal current_time) {
    // Compute Solar Radiation Pressure force in body frame
    // F_SRP = -(S0 * A * Cr / c) * (1/d^2) * s_hat_body
    // where d is sun-earth distance in meters
    
    using Real = Scalar;
    
    // Get sun vector in inertial frame
    Vector3 s_eci = helpers.earth2sun(helpers.julianDate(current_time)).normalized();
    
    // Transform to body frame
    Vector3 s_body = helpers.quatRotate(helpers.quatconj(q), s_eci);
    
    // Compute sun-earth distance (earth2sun returns normalized vector, so we use AU)
    // For simplicity, use nominal 1 AU distance
    Real d_au = static_cast<Real>(1.0);
    Real d_m = d_au * PlantParam::Sun::AU;
    
    // Compute magnitude: F = (S0 * A * Cr / c) * (1 / d^2)
    Real F_mag = (PlantParam::Sun::S0 * PlantParam::Disturbances::SRP_area * PlantParam::Disturbances::Cr) 
                 / PlantParam::Sun::c
                 * (static_cast<Real>(1.0) / (d_m * d_m))
                 * PlantParam::Disturbances::SRP_scale;
    
    // Force points away from sun (negative sign)
    Vector3 F_srp = -F_mag * s_body;
    
    return F_srp;
}

Dynamics::Vector3 Dynamics::compute_SRP_torque(const Vector3& F_srp_body, const Vector3& s_b) {
    // Compute torque: tau = r_CoP x F
    // where r_CoP is the center-of-pressure offset from CoM in body frame
    
    Vector3 tau = PlantParam::Disturbances::CoP_offset.cross(F_srp_body);
    
    return tau;
}

Dynamics::Vector3 Dynamics::compute_gravity_gradient_torque(const Vector3& r_ECI, const Vector4& q) {
    // Compute gravity-gradient torque in body frame
    // tau_gg = (3 * mu_E / r^3) * (e_r_body x (I * e_r_body))
    // where e_r is the nadir direction (from Earth center to spacecraft) in body frame
    
    using Real = Scalar;
    
    // Orbital radius
    Real r_mag = r_ECI.norm();
    
    // Nadir unit vector in inertial frame (points from Earth toward spacecraft)
    Vector3 e_r_eci = r_ECI / r_mag;
    
    // Transform nadir to body frame
    Vector3 e_r_body = helpers.quatRotate(helpers.quatconj(q), e_r_eci);
    
    // Compute gravity-gradient coefficient
    Real gg_coeff = static_cast<Real>(3.0) * mu_E / (r_mag * r_mag * r_mag) 
                  * PlantParam::Disturbances::GG_scale;
    
    // Apply inertia: I * e_r_body
    Vector3 I_e_r = I * e_r_body;
    
    // Cross product: e_r_body x (I * e_r_body)
    Vector3 tau_gg = gg_coeff * e_r_body.cross(I_e_r);
    
    return tau_gg;
}

Dynamics::Scalar Dynamics::compute_atmospheric_density(const Vector3& r_ECI) {
    // Compute atmospheric density using exponential model
    // rho = rho0 * exp(-(h - h0) / H)
    // where h is altitude above Earth surface
    
    using Real = Scalar;
    
    // Get altitude (r - r_E)
    Real r_mag = r_ECI.norm();
    Real h = r_mag - r_E;
    
    // Altitude is below minimum (below sea level equivalent) - clamp to sea level
    if (h < PlantParam::Disturbances::h0_drag) {
        h = PlantParam::Disturbances::h0_drag;
    }
    
    // Exponential atmosphere model
    Real exponent = -(h - PlantParam::Disturbances::h0_drag) / PlantParam::Disturbances::H_scale;
    Real rho = PlantParam::Disturbances::rho0_drag * std::exp(exponent);
    
    return rho;
}

Dynamics::Vector3 Dynamics::compute_drag_force(const Vector3& r_ECI, const Vector3& v_ECI) {
    // Compute atmospheric drag force
    // F_drag = -0.5 * rho * v^2 * Cd * A * v_hat
    
    using Real = Scalar;
    
    // Get velocity magnitude
    Real v_mag = v_ECI.norm();
    
    // Avoid division by zero
    if (v_mag < static_cast<Real>(1e-6)) {
        return Vector3::Zero();
    }
    
    // Compute atmospheric density at current altitude
    Real rho = compute_atmospheric_density(r_ECI);
    
    // Compute drag magnitude: 0.5 * rho * v^2 * Cd * A
    Real drag_mag = static_cast<Real>(0.5) * rho * v_mag * v_mag 
                   * PlantParam::Disturbances::Cd 
                   * PlantParam::Disturbances::A_drag
                   * PlantParam::Disturbances::drag_scale;
    
    // Drag opposes velocity: F_drag = -drag_mag * v_hat
    Vector3 v_hat = v_ECI / v_mag;
    Vector3 F_drag = -drag_mag * v_hat;
    
    return F_drag;
}

Dynamics::Vector4 Dynamics::compute_motor_current_dynamics(const Vector4& V_cmd, const Vector4& omega_wheel, const Vector4& i_motor) {
    // Electrical motor current dynamics
    // di/dt = (V - Ke*omega - R*i) / L
    // where:
    //   V = applied voltage (derived from torque command)
    //   Ke = back-EMF constant [V/(rad/s)]
    //   R = phase resistance [ohm]
    //   L = phase inductance [H]
    
    using Real = Scalar;
    
    Real Ke = PlantParam::Actuators::WheelMotor::Ke;
    Real R = PlantParam::Actuators::WheelMotor::R;
    Real L = PlantParam::Actuators::WheelMotor::L;
    
    Vector4 di_dt = Vector4::Zero();
    
    for (int i = 0; i < 4; ++i) {
        Real V = V_cmd(i);
        Real omega = omega_wheel(i);
        Real current = i_motor(i);
        
        // Voltage across motor = V_applied - back_EMF - resistive_drop
        Real V_net = V - Ke * omega - R * current;
        
        // di/dt = V_net / L
        di_dt(i) = V_net / L;
    }
    
    return di_dt;
}

Dynamics::Vector4 Dynamics::compute_wheel_friction(const Vector4& omega_wheel) const {
    Vector4 tau_fric = Vector4::Zero();
    Scalar b = PlantParam::Actuators::WheelDynamics::b_viscous;
    Scalar tau_c = PlantParam::Actuators::WheelDynamics::tau_coulomb;
    Scalar omega_eps = PlantParam::Actuators::WheelDynamics::omega_eps;

    for (int i = 0; i < 4; ++i) {
        Scalar sign = std::tanh(omega_wheel(i) / omega_eps);
        tau_fric(i) = b * omega_wheel(i) + tau_c * sign;
    }

    return tau_fric;
}

Dynamics::Vector4 Dynamics::compute_wheel_ripple(const Vector4& omega_wheel) const {
    Vector4 tau_ripple = Vector4::Zero();
    Scalar amp = PlantParam::Actuators::WheelDynamics::ripple_amp;

    for (int i = 0; i < 4; ++i) {
        tau_ripple(i) = amp * std::sin(ripple_phase(i));
    }

    return tau_ripple;
}