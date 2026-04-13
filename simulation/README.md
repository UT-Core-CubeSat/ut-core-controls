# Simulation Build Instructions

This folder contains the shared build entry point for the simulation project.
Use the `USE_ORBIT` CMake option to select between the two code bases:

- `OFF` (default): builds the air bearing demo from `../airBearingDemo/ADCSCore`
- `ON`: builds the orbit simulation from `../forOrbit/ADCSCore`

## Build steps

1. Create and enter a build directory:

(From the ut-core-controls parent folder)
```bash
mkdir -p simulation/build
cd build
```

2. Configure the project:

(Navigate to the simulation/build folder: cd simulation/build)

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

After building, run the shared `PlantSim` executable:

**On Windows (Visual Studio generator):**
```powershell
# From simulation/build/
.\Debug\PlantSim.exe
```

**On Linux/macOS:**
```bash
# From simulation/build/
./PlantSim
```

**Note:** The executable location depends on your CMake generator and build type:
- Visual Studio (Windows): `simulation/build/Debug/PlantSim.exe` (or `Release/` if built in Release mode)
- Unix Makefiles: `simulation/build/PlantSim`

## Notes

- The build uses the source code from either `airBearingDemo/ADCSCore` or `forOrbit/ADCSCore`.
- `plant_main.cpp` is compiled into the `PlantSim` executable during the build.
- If you switch between scenarios, rerun CMake in the same build directory with the desired `USE_ORBIT` setting, or delete the build directory and recreate it.
