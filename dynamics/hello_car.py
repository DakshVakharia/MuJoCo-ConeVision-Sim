"""Stage 2: Hello car - verify vehicle can step on flat terrain for 5 seconds."""

import os
import sys

# FIX: Add chrono Library\bin to DLL search path
dll_path = r"C:\Users\daksh\miniforge3\envs\chrono\Library\bin"
os.add_dll_directory(dll_path)
os.environ['PATH'] = dll_path + ";" + os.environ.get('PATH', '')

import pychrono.core as chrono
import pychrono.vehicle as veh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vehicle import build_vehicle


def main():
    print("Stage 2: Hello-Car - Basic Vehicle Stepping")
    print("=" * 60)

    # Create a temporary system (vehicle creates its own internally)
    temp_system = chrono.ChSystemSMC()

    # Build vehicle and terrain
    print("Building vehicle (sedan) and terrain...")
    vehicle, terrain = build_vehicle(temp_system, "sedan", init_pos=[0, 0, 0.5], init_yaw=0)

    print("Vehicle created successfully")
    print("Terrain created successfully")

    # Get the actual system from the vehicle
    sys_mbs = vehicle.GetSystem()

    # Step the simulation
    step_size = 0.001  # 1 ms
    sim_time = 5.0    # 5 seconds
    num_steps = int(sim_time / step_size)

    print(f"Stepping for {sim_time} s ({num_steps} steps at {step_size*1000} ms)...")
    print()

    for i in range(num_steps):
        sys_mbs.DoStepDynamics(step_size)
        t = i * step_size

        pos = vehicle.GetChassis().GetPos()
        vel = vehicle.GetChassis().GetLinearVelocity()

        if i % 1000 == 0:
            print(f"  t={t:6.2f}s: pos=[{pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:6.3f}] m, "
                  f"vel=[{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}] m/s")

    print()
    print("Stage 2: PASSED - Vehicle stepped successfully for 5 seconds")
    print("=" * 60)


if __name__ == "__main__":
    main()
