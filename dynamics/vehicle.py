"""Build a Chrono vehicle with terrain and powertrain."""

import pychrono.core as chrono
import pychrono.vehicle as veh


def build_vehicle(system, vehicle_name, init_pos, init_yaw):
    """
    Build a WheeledVehicle with engine, transmission, and tires on a RigidTerrain.

    Args:
        system: The Chrono system (not used; vehicle creates its own).
        vehicle_name (str): Name of vehicle (e.g., 'sedan').
        init_pos (tuple): [x, y, z] initial position in metres.
        init_yaw (float): Initial heading in radians (yaw).

    Returns:
        tuple: (vehicle, terrain) where vehicle is a WheeledVehicle.
    """
    # Create vehicle from JSON specification
    vehicle_json = veh.GetVehicleDataFile(f'{vehicle_name}/vehicle/{vehicle_name.capitalize()}_Vehicle.json')
    vehicle = veh.WheeledVehicle(vehicle_json, chrono.ChContactMethod_SMC)

    # Initialize at specified pose
    quat = chrono.QuatFromAngleZ(init_yaw)
    vehicle.Initialize(
        chrono.ChCoordsysd(
            chrono.ChVector3d(*init_pos),
            quat
        )
    )
    vehicle.GetChassis().SetFixed(False)

    # Set all visualizations to NONE (headless)
    vehicle.SetChassisVisualizationType(chrono.VisualizationType_NONE)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_NONE)
    vehicle.SetSteeringVisualizationType(chrono.VisualizationType_NONE)
    vehicle.SetWheelVisualizationType(chrono.VisualizationType_NONE)

    # Initialize powertrain
    engine_json = veh.GetVehicleDataFile(f'{vehicle_name}/powertrain/{vehicle_name.capitalize()}_EngineSimpleMap.json')
    transmission_json = veh.GetVehicleDataFile(f'{vehicle_name}/powertrain/{vehicle_name.capitalize()}_AutomaticTransmissionSimpleMap.json')

    engine = veh.ReadEngineJSON(engine_json)
    transmission = veh.ReadTransmissionJSON(transmission_json)
    powertrain = veh.ChPowertrainAssembly(engine, transmission)
    vehicle.InitializePowertrain(powertrain)

    # Initialize tires for all wheels
    tire_json = veh.GetVehicleDataFile(f'{vehicle_name}/tire/{vehicle_name.capitalize()}_TMeasyTire.json')
    for axle in vehicle.GetAxles():
        for wheel in axle.GetWheels():
            tire = veh.ReadTireJSON(tire_json)
            vehicle.InitializeTire(tire, wheel, chrono.VisualizationType_NONE)

    # Set collision system
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Build terrain (rigid, flat)
    terrain = veh.RigidTerrain(vehicle.GetSystem())

    # Create a large flat patch
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(0.9)
    mat.SetRestitution(0.01)
    mat.SetYoungModulus(2e7)

    # Add a large rectangular patch centered at origin
    patch = terrain.AddPatch(mat,
                             chrono.CSYSNORM,  # Identity transform at origin
                             400.0, 400.0)  # 400m x 400m patch
    terrain.Initialize()

    return vehicle, terrain
