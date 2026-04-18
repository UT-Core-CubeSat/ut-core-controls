# UT Core Controls

Flight software and simulation assets for ADCS development.

## Fast Path: SIL to HIL (Zephyr)

1. Pick one ADCS stack:
   - `airBearingDemo/ADCSCore/components/` (bench + air-bearing)
   - `forOrbit/ADCSCore/components/` (orbit mission modes)
2. Copy only that `components/` folder into your Zephyr app.
3. Create one `ADCS::Core` instance at boot.
4. In a single control thread, call `Core::update()` at a fixed rate (25 ms nominal).
5. Send `wheel_torque` and `mtq_face_current` directly to drivers.
6. Use `current_mode`, `attitude_est`, and `rate_est` for telemetry.

For simulation setup, see `simulation/README.md`.

---

## Air-Bearing API (Current)

Source of truth:
- `airBearingDemo/ADCSCore/components/ADCSCore.hpp`
- `airBearingDemo/ADCSCore/components/ADCSCore.cpp`

### Inputs

`ADCS::SensorData`
- `unix_time` [s]
- `accelerometer` [m/s^2]
- `gyro` [rad/s]
- `magnetometer` [T]
- `wheel_speeds` [rad/s]

`ADCS::Command`
- `mode` in `{ OFF, SAFE, BEARING }`

### Outputs

`ADCS::AdcsOutput`
- Actuation:
  - `wheel_torque` [N*m]
  - `mtq_dipole` [A*m^2]
  - `mtq_face_current` [A] in order `[I_Xpos, I_Xneg, I_Ypos, I_Yneg]`
  - `mtq_face_b_ref` [T] per face
- Telemetry:
  - `attitude_est` (quaternion)
  - `rate_est` [rad/s]
  - `estimator_valid`
  - `current_mode`
- Debug/SIL fields:
  - `reference`, `states_m`, `states_hat`
  - innovation fields

### Important Mode Behavior

- `OFF`: outputs zero control.
- `SAFE`: B-dot detumble path.
- `BEARING`: NDI + wheel desaturation path.
- BEARING has an internal entry latch: controller stays in detumble until body rates are below threshold on all 3 axes.
- Always use `out.current_mode` as the active mode (not just commanded mode).

---

---

## Motor Driver API (Actuator MCU)

The `MotorDriver` class serves as the high-speed bridge between the ADCS torque commands and the PWM hardware. It implements a cascaded control law that translates a requested torque into a duty cycle using a physics-based integrator, a linear RPM-to-Duty-Cycle mapping, and a PD feedback loop.

### Core Logic: Torque to Duty Cycle
The driver treats the commanded torque ($\tau_{cmd}$) as a constant acceleration request. It maintains a persistent internal target speed ($\omega_{target}$) that integrates this torque over time:
$$\omega_{target}[k] = \omega_{target}[k-1] + \left( \frac{\tau_{cmd}}{I_{wheel}} \right) \cdot dt$$
This target generates a feed-forward duty cycle via experimental RPM mapping, while a PD loop corrects for sensor-measured errors and acceleration lag.

### Integration and Usage

This class is designed to run on the Actuator/Motor MCU in a high-frequency loop (500 Hz – 1 kHz). The `dt` passed to the function must be the period of this local high-speed loop.

```cpp
#include "MotorDriver.hpp"

static MotorDriver g_driver;

void motor_loop_1000hz() {
    float dt = 0.001f; // 1kHz loop period
    
    // 1. Read current physical wheel speeds from sensors (rad/s)
    Param::Vector4 omega_measured = read_wheel_encoders_rads();
    
    // 2. Fetch latest ADCS torque and mission mode from CAN bus
    Param::Vector4 tau_from_can = get_latest_can_torque_nm();
    ADCS::MissionMode mode_from_can = get_latest_can_mode(); 
    
    // 3. Compute Duty Cycles (-1.0 to 1.0)
    Param::Vector4 duty_cycles = g_driver.computeMotorCommands(
        tau_from_can, 
        omega_measured, 
        dt, 
        mode_from_can
    );
    
    // 4. Update PWM hardware
    for (int i = 0; i < 4; ++i) {
        set_motor_pwm(i, duty_cycles(i));
    }
}
```
## Zephyr Control Loop Template (Air-Bearing)

```cpp
#include <ADCSCore.hpp>
#include <zephyr/kernel.h>

static ADCS::Core g_adcs;

void adcs_thread(void*, void*, void*) {
    constexpr uint32_t PERIOD_MS = 25;  // 40 Hz

    while (true) {
        const int64_t t0 = k_uptime_get();

        ADCS::SensorData s{};
        s.unix_time = get_time_seconds_monotonic();
        s.accelerometer = read_accel_mps2();
        s.gyro = read_gyro_rads();
        s.magnetometer = read_mag_tesla();
        s.wheel_speeds = read_wheel_speed_rads();

        ADCS::Command cmd{};
        cmd.mode = get_mode_from_cdh();  // OFF / SAFE / BEARING

        ADCS::AdcsOutput out = g_adcs.update(s, cmd);

        // Actuators
        for (int i = 0; i < 4; ++i) {
            set_wheel_torque_nm(i, out.wheel_torque(i));
        }
        set_mtq_face_current_a(0, out.mtq_face_current(0));  // +X
        set_mtq_face_current_a(1, out.mtq_face_current(1));  // -X
        set_mtq_face_current_a(2, out.mtq_face_current(2));  // +Y
        set_mtq_face_current_a(3, out.mtq_face_current(3));  // -Y

        // Telemetry
        publish_adcs_status(out.current_mode, out.estimator_valid,
                            out.attitude_est, out.rate_est);

        const uint32_t elapsed = static_cast<uint32_t>(k_uptime_get() - t0);
        if (elapsed < PERIOD_MS) {
            k_msleep(PERIOD_MS - elapsed);
        }
    }
}
```

---

## Integration Rules (Do Not Skip)

1. `unix_time` must be monotonic and in seconds.
2. Units must match API exactly (gyro and wheel speeds in rad/s).
3. Call `Core::update()` from one thread only (not re-entrant).
4. Prefer explicit `Command` every cycle; do not rely on default command overload in production.
5. Gate mission transitions on `out.current_mode` to reflect true internal state.

---

## SIL to HIL Checklist

1. Verify SAFE mode generates expected detumble commands.
2. Command BEARING and confirm `current_mode` eventually reports BEARING after rates settle.
3. Confirm actuator sign conventions on all 4 wheel channels and all 4 MTQ face channels.
4. Verify `estimator_valid` stays true during nominal operation.
5. Re-run SIL plots from `simulation/` after any actuator-frame or sign change.

---

## Repository Layout

- `airBearingDemo/ADCSCore/components/` embedded ADCS for air-bearing bench
- `forOrbit/ADCSCore/components/` embedded ADCS for orbit profiles
- `simulation/` SIL harness, CMake entry point, MATLAB plotting and analysis
