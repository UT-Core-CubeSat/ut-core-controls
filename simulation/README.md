# Simulation Build Instructions

This folder contains the shared build entry point for the simulation project.
Use the `USE_ORBIT` CMake option to select between the two code bases:

- `OFF` (default): builds the air bearing demo from `../airBearingDemo/ADCSCore`
- `ON`: builds the orbit simulation from `../forOrbit/ADCSCore`

## Build steps

1. Create and enter a build directory:

```bash
mkdir -p build
cd build
```

2. Configure the project:

- Air bearing demo (default):

```bash
cmake ..
```

- Orbit simulation:

```bash
cmake .. -DUSE_ORBIT=ON
```

3. Build the executable:

```bash
cmake --build .
```

## Run the simulation

After building, run the shared `PlantSim` executable from `simulation/build`:

```bash
./PlantSim
```

On Windows, use:

```powershell
PlantSim.exe
```

## Notes

- The build uses the source code from either `airBearingDemo/ADCSCore` or `forOrbit/ADCSCore`.
- `plant_main.cpp` is compiled into the `PlantSim` executable during the build.
- If you switch between scenarios, rerun CMake in the same build directory with the desired `USE_ORBIT` setting, or delete the build directory and recreate it.
