#include "core_ControllerBDot.hpp"
#include "core_Saturate.hpp"
#include <cmath>

//this file has been recommented by Averie on 10/9/2026
// PURPOSE: Implements a B-dot detumbling controller

// A spacecraft that is rotating experiences changes in the
// magnetic field measured in its body frame. The B-dot control
// law uses this change to estimate rotational motion and command
// magnetorquers to reduce the spacecraft's angular velocity.

// High level process:
/// Extract mag field and gyro measuremetns
/// Estimate magnetic field derivitave (hence BDOT)
/// filter derivitave to reduce noise 
/// Combine derivitave estimate with gyro estimate (b_fuse)
/// calculate the needed magnetic dipole moment
/// remove the dipole componet parallel to the mag field
/// saturate command and disable body- z component



// Constructor 
ControllerBDot::ControllerBDot()
    : // Initialize Members 
    m_min(Param::Actuators::m_min),  //Minimum allowable actuator magnetic moment
    m_max(Param::Actuators::m_max),  //Maximum allowable actuator magnetic moment
    K_Bdot(Param::Controller::K_Bdot),  //bdot control law gain
    B_prev(Vector3::Zero()),            // previous update's mag-field measurement
    B_dot(Vector3::Zero()),             // current est. of the mag-field derivitate (BDOT)
    alpha_Bdot(Param::Controller::alpha_BDot),  //Filter coeff for numerical BDOT est 
    Bdot_num_filt(Vector3::Zero()),             //stored filtered numerical BDOT
    beta_fuse(Param::Controller::beta_fuse)     //weight used to combine gyro measurements and BDOT
{
///Internal vectors all start at zero. If b_prev is 0, it is assumed first update
}

// Methods

/// MAIN BDOT CONTROLLER UPDATE
// Calculates the magnetic dipole moment for one update cycle

// Input: Measurements (vector with gyro and mag field), states_hat (11 element state vector, unused by this controller), dt (time since previous update)
// Output: Vector3: 3 axis magnetic dipole command.

ControllerBDot::Vector3 ControllerBDot::update(const Measurements& measurements, const Param::Vector11& states_hat, Scalar dt)
{
    // Extract magnetic field measurement

    // Measurement vector stores 3-5 angular velocity from gyro
    // elements 6-8 mag-field measurement

    Vector3 B_now = measurements.segment<3>(6); // Assuming B field is in elements 6-8
    //current magfield vector
    Vector3 omega_meas = measurements.segment<3>(3); // Assuming angular velocity is in elements 3-5
    //current angular velocity
    Vector3 m_tilde;
    // magnetic dipole command from bdot control law before saturation
    Vector3 Bh;
    // unit vector pointing in the direction of the measured field 
    Vector3 m_sat;
    //final saturated magnetic dipole command
    (void)states_hat;

    if (B_prev.isZero()) {   //if first update 
        B_prev = B_now;
        B_dot = Vector3::Zero();
        return Vector3::Zero();
    }


    //validate time step, must be non-zero and finite 
    if (!std::isfinite(dt) || dt <= static_cast<Scalar>(1e-6)) {
        return Vector3::Zero();
    }

    // RTOS Optimization: Pre-compute norm once and avoid redundant computations
    Scalar B_norm = B_now.norm();
    Scalar B_norm_threshold = static_cast<Scalar>(1e-9);
    
    // Apply low-pass filter to B_dot
    Scalar dt_inv = static_cast<Scalar>(1.0) / dt;  // Pre-compute reciprocal (faster than division)
    
    //numerical derivitave 
    //dB/dt ~~ (B_now - B_prev) /dt 
    //then apply a low-pass filter to smooth the numerical derivitabe 
    //alpha_Bdot controls the relative contribution of the
    // previous filtered value versus the new derivative.
    //
    // a arger alpha_Bdot generally retains more of the previous
    // filtered value, while smaller values respond more strongly
    // to the newest measurement. Potentially a value that can be modified for 
    // testing purposes 
    Bdot_num_filt = (static_cast<Scalar>(1) - alpha_Bdot) * (B_now - B_prev) * dt_inv + alpha_Bdot * Bdot_num_filt;
    
    

    
    B_dot = (static_cast<Scalar>(1) - beta_fuse) * (-omega_meas.cross(B_now)) + beta_fuse * Bdot_num_filt;

    ///^^beta_fuse weights the numerical derivitave against the gyro-based estimate
    //if 0: -> use only gyro based estimate
    //if 1: use only filtered numerical.
    //intermediate values blend the two estimates
    
    m_tilde = -K_Bdot * B_dot;
    //^^Applies bdot control law


    //magnetic torque is m cross product B, 
    // A magnetic dipole component parallel to B produces no
    // torque. Removing that component avoids commanding a
    // dipole component that cannot generate useful torque



    // Saturate magnetic moment and project perpendicular to B-field (rank-1 projection)
    if (B_norm > B_norm_threshold) {
        // Use pre-computed norm to avoid redundant calculation
        Scalar inv_norm = static_cast<Scalar>(1.0) / B_norm;
        Bh = B_now * inv_norm;

        // Direct rank-1 projection: m_tilde -= Bh * (Bh.dot(m_tilde))
        // This removes the parallel component; no wasted torque along B-field
        m_tilde -= Bh * Bh.dot(m_tilde);
    }
    
    m_sat = saturateSymmetric(m_tilde, m_max);
    // Four long-face panel coils provide no body-Z magnetic authority in this test setup.
    m_sat(2) = static_cast<Scalar>(0.0);
    // Update B_prev
    B_prev = B_now;
    return m_sat;
}