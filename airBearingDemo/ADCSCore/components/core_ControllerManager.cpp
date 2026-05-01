#include "core_ControllerManager.hpp"

// Constructor 
ControllerManager::ControllerManager() 
    : // Initialize Members
    NDI(),
    Bdot(),
    prev_mode(Param::PointingMode::OFF)
{

}

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
        output.tau(4) = bdot_out(0);
        output.tau(5) = bdot_out(1);
        output.tau(6) = bdot_out(2);
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
}