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

### Zephyr Integration: Complete API Reference

#### Input Structure: `ADCS::SensorData`

Pack sensor measurements at your sensor read rate (can be faster than 40 Hz; Core processes data at whatever rate you call it):

```cpp
struct SensorData {
    Param::TimeReal unix_time;          // [seconds] Timestamp from system clock or GPS
    Math::Vec<3> accelerometer;         // [m/s²] IMU gravity vector, body frame
    Math::Vec<3> gyro;                 // [rad/s] IMU angular rates, body frame
    Math::Vec<3> magnetometer;         // [T] Magnetometer field vector, body frame
    Math::Vec<4> wheel_speeds;         // [rad/s] Reaction wheel speeds from encoders
};
```

**Memory layout & sizes**: 
- `unix_time`: 8 bytes (double precision) 
- `accelerometer`: 12 bytes (3 × float32)
- `gyro`: 12 bytes
- `magnetometer`: 12 bytes
- `wheel_speeds`: 16 bytes (4 × float32)
- **Total: 60 bytes** (one SensorData packet)

**Requirements**:
- `unix_time` must be monotonically increasing; used for `dt = current_time - last_time` in Core
- First call: `dt` is set to 0; observer skips first propagation
- Subsequent calls: any positive dt > 1 µs accepted. Standard: 25 ms (~40 Hz)
- Sensor data should be in **body-fixed frame**: X forward, Y right, Z down (or your convention—must be consistent)
- Ensure IMU gyro bias is removed in your sensor fusion preprocessing if possible

#### Command Structure: `ADCS::Command`

Tells Core what mission mode to run; comes from CDH or ground command:

```cpp
enum class MissionMode {
    SAFE,      // All actuators OFF; observer still runs (useful for commissioning)
    DETUMBLE,  // B-dot active; dumping angular momentum via magnetic field
    POINT      // NDI active; tracking attitude reference (see note below)
};

struct Command {
    MissionMode mode;
    // Default constructor initializes to POINT
};
```

**Note on POINT mode**: Currently hardcoded to identity quaternion + zero rate reference in air-bearing configuration. To use NDI pointing, integrate a reference generator (see `forOrbit/ADCSCore/components/core_ReferenceGenerator.hpp`).

#### Output Structure: `ADCS::AdcsOutput`

Everything your software engineer needs in one struct, returned from `Core::update()`:

```cpp
struct AdcsOutput {
    // ===== ACTUATOR COMMANDS (what to send to hardware) =====
    Math::Vec<4> wheel_torque;         // [N·m] Reaction wheel torque commands (4 wheels)
    Math::Vec<3> mtq_dipole;           // [A·m²] Body-frame magnetic dipole (internal representation)
    
    // ===== PER-FACE MTQ INTERFACE (optimized for firmware) =====
    Math::Vec<4> mtq_face_current;     // [A] Per-face coil currents: [I_Xpos, I_Xneg, I_Ypos, I_Yneg]
    Math::Vec<4> mtq_face_b_ref;       // [T] Reference field magnitude each face produces
    
    // ===== TELEMETRY (what to log/transmit on CAN) =====
    Math::Vec<4> attitude_est;         // [quaternion] Estimated attitude (q_w, q_x, q_y, q_z)
    Math::Vec<3> rate_est;             // [rad/s] Estimated body angular rate
    bool estimator_valid;              // true if state estimate has no NaN
    MissionMode current_mode;          // Echo of command.mode sent in
    
    // ===== DEBUG / SIM ONLY (not for production firmware) =====
    Math::Vec<10> reference;           // Reference trajectory (zero for air bearing)
    Math::Vec<7> states_m;             // Model states from controller (for equivalence checking)
    Math::Vec<11> states_hat;          // Full estimated state (for plotting/logging)
};
```

**Memory layout & sizes**:
- `wheel_torque`: 16 bytes
- `mtq_dipole`: 12 bytes
- `mtq_face_current`: 16 bytes
- `mtq_face_b_ref`: 16 bytes
- `attitude_est`: 16 bytes
- `rate_est`: 12 bytes
- `estimator_valid`: 1 byte (padded to 4)
- `current_mode`: 4 bytes (enum)
- `reference`: 40 bytes
- `states_m`: 28 bytes
- `states_hat`: 44 bytes
- **Total: ~224 bytes** (for one output packet; ~180 bytes if you ignore debug fields)

**Field-by-field guidance for firmware integration**:

| Field | Use Case | Range/Units | Access Pattern |
|-------|----------|-------------|-----------------|
| `wheel_torque[4]` | Reaction wheel driver | ±13 mN·m | `ctrl.wheel_torque(i)` or `ctrl.wheel_torque(0..3)` |
| `mtq_face_current[4]` | Magnetorquer driver PC boards | ±0.5 A | Direct: `ctrl.mtq_face_current(0)` = I_Xpos |
| `mtq_face_b_ref[4]` | Telemetry / logging | 0 to 10 µT typical | Verify expected field on each face |
| `attitude_est[4]` | CAN telemetry | unit quaternion | Transmit all 4 components; receiver normalizes |
| `rate_est[3]` | CAN telemetry | rad/s | Body rates (gyro bias-corrected estimate) |
| `estimator_valid` | Health check | boolean | Skip CAN TX if false; log event |
| `current_mode` | Status echo | 0=SAFE, 1=DETUMBLE, 2=POINT | Verify command took effect |

#### Complete Integration Example

```cpp
#include <ADCSCore.hpp>
#include <zephyr/kernel.h>

// Global instance - created once at boot
static ADCS::Core adcs_core;

// Thread-safe CAN TX helpers (pseudo-code; adapt to your CAN driver)
void tx_actuator_commands(const ADCS::AdcsOutput& ctrl) {
    uint8_t msg[64];
    msg[0] = (uint8_t)ctrl.current_mode;                    // Status byte
    memcpy(&msg[1], &ctrl.wheel_torque(0), 16);             // 4 wheel torques
    memcpy(&msg[17], &ctrl.mtq_face_current(0), 16);        // 4 MTQ currents
    // CAN TX at ID 0x123
    send_can_frame(0x123, msg, 33);
}

void tx_telemetry(const ADCS::AdcsOutput& ctrl) {
    if (!ctrl.estimator_valid) {
        LOG_ERR("ADCS: estimator invalid");
        return;
    }
    uint8_t msg[32];
    memcpy(&msg[0], &ctrl.attitude_est(0), 16);             // Quaternion
    memcpy(&msg[16], &ctrl.rate_est(0), 12);                // Rate
    send_can_frame(0x124, msg, 28);
}

void adcs_control_thread(void *arg1, void *arg2, void *arg3) {
    (void)arg1; (void)arg2; (void)arg3;
    
    // Timing: run at 40 Hz (~25 ms period)
    static const uint32_t ADCS_PERIOD_MS = 25;
    
    while (true) {
        k_uptime_t start = k_uptime_get();
        
        // 1. Collect sensor data at thread rate (can be faster if chained from sensor drivers)
        //    Example: if IMU ISR fires at 200 Hz, buffer samples and use latest here
        ADCS::SensorData sensors = {0};
        sensors.unix_time = get_system_time_seconds();          // Your epoch (e.g., GPS time)
        sensors.accelerometer(0) = read_imu_ax();
        sensors.accelerometer(1) = read_imu_ay();
        sensors.accelerometer(2) = read_imu_az();
        sensors.gyro(0) = read_gyro_x();
        sensors.gyro(1) = read_gyro_y();
        sensors.gyro(2) = read_gyro_z();
        sensors.magnetometer(0) = read_mag_x();
        sensors.magnetometer(1) = read_mag_y();
        sensors.magnetometer(2) = read_mag_z();
        sensors.wheel_speeds(0) = read_wheel_0_speed();
        sensors.wheel_speeds(1) = read_wheel_1_speed();
        sensors.wheel_speeds(2) = read_wheel_2_speed();
        sensors.wheel_speeds(3) = read_wheel_3_speed();
        
        // 2. Prepare command (typically from received CAN message or internal state machine)
        ADCS::Command cmd;
        cmd.mode = get_mission_mode_from_cdh();  // SAFE, DETUMBLE, or POINT
        
        // 3. Run Core (black box - no control law knowledge needed)
        // Execution time: ~2-4 ms on ARM Cortex-M4 @ 168 MHz (includes Kalman update, NDI/B-dot)
        ADCS::AdcsOutput ctrl = adcs_core.update(sensors, cmd);
        
        // 4. Extract and send commands to hardware drivers
        //    Per-face MTQ currents are ready for driver PCBs:
        float I_Xpos = ctrl.mtq_face_current(0);  // Send to +X coil driver
        float I_Xneg = ctrl.mtq_face_current(1);  // Send to -X coil driver
        float I_Ypos = ctrl.mtq_face_current(2);  // Send to +Y coil driver
        float I_Yneg = ctrl.mtq_face_current(3);  // Send to -Y coil driver
        set_mtq_current(MTQ_XPOS, I_Xpos);
        set_mtq_current(MTQ_XNEG, I_Xneg);
        set_mtq_current(MTQ_YPOS, I_Ypos);
        set_mtq_current(MTQ_YNEG, I_Yneg);
        
        //    Reaction wheel torques:
        for (int i = 0; i < 4; ++i) {
            set_wheel_torque(i, ctrl.wheel_torque(i));
        }
        
        // 5. Send CAN telemetry and actuator status
        tx_actuator_commands(ctrl);
        tx_telemetry(ctrl);
        
        // 6. Log diagnostics (optional; can be conditionally compiled out for flight)
        #ifdef ADCS_DIAGNOSTICS
        LOG_INF("ADCS: q = [%.3f, %.3f, %.3f, %.3f], rate = [%.3f, %.3f, %.3f] rad/s",
                ctrl.attitude_est(0), ctrl.attitude_est(1), ctrl.attitude_est(2), ctrl.attitude_est(3),
                ctrl.rate_est(0), ctrl.rate_est(1), ctrl.rate_est(2));
        LOG_DBG("ADCS: I_mtq = [%.3f, %.3f, %.3f, %.3f] A",
                I_Xpos, I_Xneg, I_Ypos, I_Yneg);
        #endif
        
        // 7. Sleep until next period
        uint32_t elapsed = k_uptime_get() - start;
        if (elapsed < ADCS_PERIOD_MS) {
            k_msleep(ADCS_PERIOD_MS - elapsed);
        } else {
            LOG_WRN("ADCS: overrun (%u ms > %u ms)", elapsed, ADCS_PERIOD_MS);
        }
    }
}

// Boot-time initialization
void adcs_init(void) {
    // Core is already constructed as static global; observer and filters initialize in ctor
    LOG_INF("ADCS Core initialized");
}

// Optional: High-speed sensor thread (e.g., 200 Hz IMU)
void imu_sensor_thread(void *arg1, void *arg2, void *arg3) {
    // Read IMU at fast rate, buffer into shared struct for ADCS thread to consume
    // Prevents blocking ADCS on slow I/O operations
}
```

#### Memory & Performance Summary

| Aspect | Value | Notes |
|--------|-------|-------|
| **Per-call memory** | ~220 bytes | SensorData + AdcsOutput + internal state (60+220+~11×4+6×6 matrices) |
| **Execution time** | 2–4 ms @ 168 MHz | Includes Kalman EKF, Bdot/NDI control, matrix inversions |
| **Timing jitter** | ±0.5 ms typical | Deterministic; no dynamic allocation in hot path |
| **Stack depth** | ~4 KB total | Observer + Controller + helper functions (pre-allocated buffers) |
| **Control loop latency** | <dt (25 ms typical) | One-step delay due to batch processing |
| **Slack for 40 Hz** | ~21 ms | 25 ms period - 2 ms execution = plenty of margin |
| **How to sample faster** | Call `adcs_core.update()` at desired rate | Dt is computed internally; faster calls = higher bandwidth |

**RTOS Guidelines**:
- Use a dedicated thread (2-4 KB stack) for the ADCS control loop
- Priority: medium (below hard realtime ISRs; above non-critical I/O)
- No dynamic allocation; all buffers pre-allocated by observer and controller on construction
- Thread-safe: Core is *not* re-entrant; serialize calls within a single thread or mutexed region
- Fastest sampling recommended: 50–100 Hz (2 KB stack + margin for OS overhead)
