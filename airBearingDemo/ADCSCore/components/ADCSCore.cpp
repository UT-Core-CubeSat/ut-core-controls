#include "ADCSCore.hpp"
#include <cmath>

//This file has been recommented by Averie on 10/9/2026
namespace ADCS {


/// constructor: create observer, create controller, first instance of update fucntion
/// starts previous timestep at 0, creates empty measurement vecrots
/// doesnt arm bearing mode until the angular rates are low enough 
Core::Core()
    : observer_(), controller_(), first_update(true), last_time(static_cast<Param::TimeReal>(0.0)),
    workspace_meas_(Param::Vector13::Zero()), workspace_ref_(Param::Vector10::Zero()),
    bearing_mode_armed(false)
{
}



/// update intakes Accelerometer, gyro, meagnet, reaction wheel speeds, and time 
/// as well as one of four commands: OFF, DETUMBLE, MOTOR, BOTH


AdcsOutput Core::update(const SensorData& sensors, const Command& command)
{

    // Compute variable dt (how much time has passed before last ADCS Update)
    //if first update, sets dt to zero
    Param::TimeReal dt;
    if (first_update) {
        dt = static_cast<Param::TimeReal>(0.0);
        first_update = false;
    } else {
        dt = sensors.unix_time - last_time;
    }
    last_time = sensors.unix_time;

    // 1. Pack measurements for observer (reuse workspace buffer to avoid per-call allocation)
    // [0-2]: accelerometer (gravity in body frame)
    // [3-5]: gyro (body rates)
    // [6-8]: magnetometer (B-field in body frame)
    // [9-12]: wheel speeds
    workspace_meas_.setSegment(0, sensors.accelerometer);
    workspace_meas_.setSegment(3, sensors.gyro);
    workspace_meas_.setSegment(6, sensors.magnetometer);
    workspace_meas_.setSegment(9, sensors.wheel_speeds);
    //^^the above code makes a vector containing all the measurements
    // vector is 12x1 


    // 2. Run observer
    Param::Real dt_scalar = static_cast<Param::Real>(dt);  // Safe: dt is small (0.025s), no precision loss
    Param::Vector11 states_hat = observer_.update(workspace_meas_, sensors.unix_time, dt_scalar);
    //^^^this line runs the observer
    // Luenberger produces estimated state and states_hat


    // 3. Build reference (reuse workspace buffer; initialize for identity pointing)
    workspace_ref_ = Param::Vector10::Zero();
    workspace_ref_(0) = 1; // unit quaternion, no rotation
    // start with identity quaternion is [1, 0, 0, 0]
    if (command.mode != MissionMode::BOTH) {
        bearing_mode_armed = false;
    }

    // if not both mode, reset the armed flag?


    //If mission says a mode, controller does that mode.
    Param::PointingMode mode = Param::PointingMode::OFF;
    if (command.mode == MissionMode::OFF) {
        mode = Param::PointingMode::OFF;
    } else if (command.mode == MissionMode::DETUMBLE) {
        mode = Param::PointingMode::DETUMBLE;
    } else if (command.mode == MissionMode::MOTOR) {
        mode = Param::PointingMode::MOTOR;
    } else if (command.mode == MissionMode::BOTH) {
        if (!bearing_mode_armed) {
            Param::Real w_thresh = Param::Controller::bearing_entry_axis_rate_threshold;
            bool rates_ready = (std::abs(sensors.gyro(0)) <= w_thresh) &&
                               (std::abs(sensors.gyro(1)) <= w_thresh) &&
                               (std::abs(sensors.gyro(2)) <= w_thresh);
            if (rates_ready) {
                bearing_mode_armed = true;
            }
        }
        mode = bearing_mode_armed ? Param::PointingMode::BOTH : Param::PointingMode::DETUMBLE;
    }
    ///^ the last else if block is what to do when the BOTH command is recieved.
    /// If the angular rates are not low enough, it does detumbling instead
    /// if they are, it does both detumbling and motor.

    // 4. Run controller
    auto ctrl_out = controller_.update(states_hat, workspace_ref_, workspace_meas_, mode, dt_scalar);
    //controller produces an output vector matrix.
    // Use the estimated state, desired reference, measurements, and
    // current operating mode to calculate the required actuator torques.
    

    // 5. Pack output

    // Split the controller's actuator command into reaction-wheel torques
    // and the 3-axis magnetorquer dipole command.  
    ///tau =
        // [ wheel 1
        // wheel 2
        // wheel 3
        // wheel 4
        // MTQ x
        // MTQ y
        // MTQ z ]
    AdcsOutput out;
    out.wheel_torque = ctrl_out.tau.segment<4>(0);
    out.mtq_dipole = ctrl_out.tau.segment<3>(4);
    out.attitude_est = states_hat.segment<4>(6);
    out.rate_est = states_hat.segment<3>(10);
    out.estimator_valid = !states_hat.hasNaN();
    if (mode == Param::PointingMode::OFF) {
        out.current_mode = MissionMode::OFF;
    } else if (mode == Param::PointingMode::DETUMBLE) {
        out.current_mode = MissionMode::DETUMBLE;
    } else if (mode == Param::PointingMode::MOTOR) {
        out.current_mode = MissionMode::MOTOR;
    } else {
        out.current_mode = MissionMode::BOTH;
    }
    //^Convert controller mode back to mission mode
 

    // 6. Allocate body dipole to per-face currents and reference fields (black-box derivation)
    // Software engineer receives these ready-to-use without understanding control law
    auto alloc_out = mtq_allocator_.allocate(out.mtq_dipole, sensors.magnetometer);
    //takes dipole from controller and turns into magtourque commands
    
    out.mtq_face_current = alloc_out.face_current;
    out.mtq_face_b_ref = alloc_out.face_b_ref;

    // Adding equivalence variables, this is just for plotting and wouldn't be needed in orbit
    out.reference = workspace_ref_; 
    out.states_m = ctrl_out.states_m;
    out.states_hat = states_hat;
    
    // Innovation diagnostics for tuning
    out.accel_innovation = observer_.getLastAccelInnovation();
    out.accel_innovation_norm = observer_.getLastAccelInnovationNorm();
    out.mag_innovation = observer_.getLastMagInnovation();
    out.mag_innovation_norm = observer_.getLastMagInnovationNorm();
    //if innovation is large there may be a problem 

    return out;
}

AdcsOutput Core::update(const SensorData& sensors)
{
    Command default_command;
    return update(sensors, default_command);
}

//if there is no command provided, it creates a default command
//and uses the main update function

void Core::reset() {
    observer_ = ObserverClass();
    controller_ = ControllerManager();
    first_update = true;
    last_time = static_cast<Param::TimeReal>(0.0);
    bearing_mode_armed = false;
}


//resets observer, controller, says next update is the first, clears time,
//disarms BOTH


} // namespace ADCS