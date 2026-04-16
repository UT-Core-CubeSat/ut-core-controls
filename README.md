# UT Core Controls

This repository contains flight software components for an Attitude Determination and Control System (ADCS) and related simulation support.

## Primary Usage

For embedded ZephyrRTOS applications, the main reusable portion is the `components/` folder inside either:

- `forOrbit/ADCSCore/components/`
- `airBearingDemo/ADCSCore/components/`

These folders contain the ADCS flight software modules that can be pulled directly into a Zephyr-based project.

## Recommended Embedded Workflow

1. Clone or download this repository.
2. Copy only the `components/` directory from the folder that best matches your hardware and use case:
   - `forOrbit/ADCSCore/components/` for orbit-focused ADCS code.
   - `airBearingDemo/ADCSCore/components/` for air-bearing demo hardware.
3. Integrate the copied source files into your ZephyrRTOS application build system.
4. Add any required target-specific glue or sensor/actuator interface code in your own project.

## Simulation and SIL Verification

This repository also includes a simulation workspace for software-in-the-loop (SIL) and verification work.

- See the `simulation/` folder
- Refer to `simulation/README.md` for simulation-specific setup and usage instructions

## Repository Layout

- `airBearingDemo/ADCSCore/`
  - `components/` — embedded flight software source files for air-bearing demo use
  - `plant/` — plant model and demo harness sources
- `forOrbit/ADCSCore/`
  - `components/` — embedded flight software source files for orbit use
  - `plant/` — plant and orbital dynamics model sources
- `simulation/`
  - `README.md` — simulation setup and instructions
  - `CMakeLists.txt` — simulation build configuration
  - MATLAB scripts and animation tools

## Notes

- The `components/` directories are intended for integration into an embedded firmware project.
- The `plant/` folders are primarily used for simulation, testing, and verification.
- If you are only interested in hardware firmware, you typically do not need the `simulation/` folder.

## Air-Bearing MTQ Interface Contract

For the air-bearing detumble use case, the repository now treats magnetorquer commands with the following semantics:

- Canonical command variable: body-frame magnetic dipole `m_cmd` in `A*m^2`
- Torque generation physics: `tau_mtq = m_cmd x B_body`
- B-dot detumble authority: body `X` and `Y` are active, body `Z` is clamped to zero (`m_z = 0`) to match long-face PCB coil authority assumptions
- Control loop target rate: `40 Hz` (`Ts = 0.025 s`)

### Per-Face Allocator

**Key capability**: The Core automatically derives per-face coil currents from the internal body dipole. No firmware allocator needed.

`AdcsOutput` includes:
- `mtq_face_current[4]`: Per-face current [A] in order [I_Xpos, I_Xneg, I_Ypos, I_Yneg] — **ready to transmit on CAN**
- `mtq_face_b_ref[4]`: Per-face reference field magnitude [T] each coil produces — **for calibration telemetry**

Coil calibration constants (turns, area per face, max current) are in `core_Parameters.hpp::Actuators::Coils` and can be updated based on PCB measurements or Helmholtz cage calibration.

### Zephyr Integration Example

```cpp
// In your Zephyr application
#include <ADCSCore.hpp>

ADCS::Core adcs_core;

void adcs_control_thread() {
    while (true) {
        // 1. Collect sensors (CAN RX from hardware)
        ADCS::SensorData sensors;
        sensors.unix_time = get_timestamp();
        sensors.accelerometer = read_imu_accel();
        sensors.gyro = read_imu_gyro();
        sensors.magnetometer = read_mag();
        sensors.wheel_speeds = read_wheel_encoders();
        
        // 2. Define mission command (from CDH/ground)
        ADCS::Command cmd;
        cmd.mode = ADCS::MissionMode::DETUMBLE;  // or SAFE/POINT
        
        // 3. Run core (entire control law encapsulated as black box)
        ADCS::AdcsOutput ctrl = adcs_core.update(sensors, cmd);
        
        // 4. Pack CAN output - per-face currents READY TO TRANSMIT
        can_frame.mtq_I_Xpos = ctrl.mtq_face_current(0);
        can_frame.mtq_I_Xneg = ctrl.mtq_face_current(1);
        can_frame.mtq_I_Ypos = ctrl.mtq_face_current(2);
        can_frame.mtq_I_Yneg = ctrl.mtq_face_current(3);
        
        // Optional: log reference fields for driver feedback
        can_frame.mtq_B_Xpos_ref = ctrl.mtq_face_b_ref(0);
        // ... similar for other faces
        
        // Also available:
        // - ctrl.wheel_torque[4] for reaction wheel commands
        // - ctrl.attitude_est[4] for attitude quaternion telemetry
        // - ctrl.current_mode for mode echo
        
        can_transmit(&can_frame);
        k_msleep(25);  // 40 Hz control loop
    }
}
```
