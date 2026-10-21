#include "core_ControllerManager.hpp"


//This file was recommented by Averie on 10/9/2026 

// PURPOSE: manages the selection and execution of the ADCS control laws 


// The ControllerManager chooses which controller to run based
// on the requested pointing mode:
//
//   OFF      - No active control; all torque commands are zero.
//   DETUMBLE - Run B-dot control and command magnetorquers.
//   MOTOR    - Run NDI control and command reaction wheels.
//   BOTH     - Run NDI control and use both wheels and MTQs.


// Constructor 
ControllerManager::ControllerManager() 
    : // Initialize Members
    NDI(), //initializws NDI controller
    Bdot(), //initializes bdot
    prev_mode(Param::PointingMode::OFF) //records previous pointing mode, defaults to OFF on startup
{

}

//Main Controller update 
// INPUTS:
//   states_hat  - Eleven-element estimated state vector.
//   reference   - Ten-element controller reference vector.
//   measurements - Thirteen-element measurement vector.
//   mode        - Requested operating mode.
//   dt          - Elapsed time since the previous update [s].
//
// OUTPUT:
//   ControlOutput containing the torque command vector and
//   the controller's seven-element model-state vector.


ControllerManager::ControlOutput ControllerManager::update(const Param::Vector11& states_hat, 
                                                            const Param::Vector10& reference,
                                                            const Param::Vector13& measurements,
                                                            const Param::PointingMode mode, Param::Real dt) {
    ControlOutput output;

    // Depending on mode, select controller
    if (mode == Param::PointingMode::DETUMBLE) {
        // B-Dot Controller: magnetorquers only, wheels zero
        auto bdot_out = Bdot.update(measurements, states_hat, dt);

        output.tau = Param::Vector7::Zero();
        output.tau(4) = bdot_out(0);  //bdot x
        output.tau(5) = bdot_out(1);  //bdot y
        output.tau(6) = bdot_out(2);  //bdot z
        output.states_m = Param::Vector7::Zero();

    } else if (mode == Param::PointingMode::MOTOR) {
        // NDI Controller: reaction wheels only, MTQ zero
        auto ndi_out = NDI.update(states_hat, reference, measurements, dt);

        output.tau = Param::Vector7::Zero();
        output.tau.setSegment(0, ndi_out.tau_wheel);
        output.states_m = ndi_out.states_m;

    } else if (mode == Param::PointingMode::BOTH) {
        // NDI Controller: reaction wheels + magnetorquer wheel-momentum desaturation
        auto ndi_out = NDI.update(states_hat, reference, measurements, dt);

        output.tau.setSegment(0, ndi_out.tau_wheel);
        output.tau.setSegment(4, ndi_out.tau_mtq);
        output.states_m = ndi_out.states_m;

    } else {
        // OFF Mode: zero torques
        output.tau = Param::Vector7::Zero();
        output.states_m = Param::Vector7::Zero();
    }

    // Update previous mode
    prev_mode = mode;

    return output;

    //outpur contains torque command vector and controller seven-element model-state vector 
}