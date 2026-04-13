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
