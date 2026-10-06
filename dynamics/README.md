# Chrono Vehicle Dynamics Simulator

Simulates Formula Student vehicle dynamics using Project Chrono with a path-following driver.

## Files

- **vehicle.py**: Builds a WheeledVehicle with engine, transmission, tires, and RigidTerrain
- **drivers.py**: Path-following driver with Bezier curve path from track centerline
- **speed_profile.py**: Curvature-limited speed profile (not yet integrated)
- **run_offline.py**: Main simulation runner; outputs 1 kHz trajectory CSV
- **test_offline.py**: Unit tests for quaternion conversion and basic simulation
- **hello_car.py**: Stage 2 test (vehicle steps on flat ground for 5 seconds)

## Usage

**Setup**: Use conda environment at `C:\Users\daksh\miniforge3\envs\chrono\`

**Run simulation** (2 laps at 10 m/s):
```bash
# Git Bash or POSIX shell (required for PATH/DLL handling)
/c/Users/daksh/miniforge3/envs/chrono/python.exe dynamics/run_offline.py \
  --track generated/bundle/track.yaml \
  --laps 2 \
  --vmax 10.0 \
  --out generated/chrono/trajectory_chrono.csv
```

**Output**: 
- CSV at `generated/chrono/trajectory_chrono.csv` with columns:
  `t, x, y, z, roll, pitch, yaw, v, yaw_rate, ax, ay, az, wx, wy, wz`
- At 1 kHz sampling (1 row per ms)
- Follows `docs/trajectory_contract.md` specification

## Key Implementation Notes

1. **Vehicle loading**: Uses subdirectory JSON files (`sedan/vehicle/Sedan_Vehicle.json`, not high-level spec)
2. **Stepping loop**: Synchronize all modules, then Advance all (see demo references)
3. **Orientation**: ChQuaterniond [w,x,y,z] converted to Euler angles [roll, pitch, yaw]
4. **Specific force**: Includes gravity; body frame via R^T(a_world + g_world)
5. **Headless**: All visualization set to NONE; no Irrlicht/VSG windows
6. **DLL fix**: PATH and os.add_dll_directory() set before importing pychrono (SDL3 compatibility)

## Test

```bash
# Stage 2: basic vehicle stepping
/c/Users/daksh/miniforge3/envs/chrono/python.exe dynamics/hello_car.py

# Unit tests
/c/Users/daksh/miniforge3/envs/chrono/python.exe dynamics/test_offline.py
```

## Vehicle Models

Chrono provides several vehicles in `Library/data/vehicle/`:
- `sedan`: Passenger car (DEFAULT)
- `hmmwv`: Military vehicle
- `polaris`: ATV
- `gator`: Small utility vehicle
- `generic`: Generic vehicle

Each has subdirectories for:
- `vehicle/`: Vehicle_*.json (body, suspension, steering)
- `powertrain/`: Engine and transmission specs
- `tire/`: Tire models (TMeasy, Pac02, Rigid, Fiala)

## References

- PyChro demos: `C:\Users\daksh\miniforge3\envs\chrono\Lib\site-packages\pychrono\demos\vehicle\`
  - `demo_VEH_WheeledJSON.py`: Full JSON vehicle loading
  - `demo_VEH_SteeringController.py`: ChPathFollowerDriver example
- Chrono trajectory contract: `docs/trajectory_contract.md`
