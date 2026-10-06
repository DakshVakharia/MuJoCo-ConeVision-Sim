"""Path-following driver for Chrono vehicle."""

import pychrono.core as chrono
import pychrono.vehicle as veh
import numpy as np


def build_path_from_centerline(centerline_points):
    """
    Build a Chrono ChBezierCurve from track centerline points.

    Args:
        centerline_points: (N, 2) array of [x, y] coordinates

    Returns:
        chrono.ChBezierCurve: Closed-loop path
    """
    # Convert to 3D with z=0
    points_3d = []
    for x, y in centerline_points:
        points_3d.append(chrono.ChVector3d(float(x), float(y), 0.0))

    # Create Bezier curve from control points
    curve = chrono.ChBezierCurve(points_3d)

    return curve


def build_path_follower_driver(vehicle, centerline_points, target_speed_func):
    """
    Build a path-follower driver that follows a track centerline.

    Args:
        vehicle: WheeledVehicle object
        centerline_points: (N, 2) array of track centerline [x, y] coordinates
        target_speed_func: function(distance_along_path) -> speed in m/s

    Returns:
        veh.ChPathFollowerDriver: Configured driver
    """
    # Build the path
    path = build_path_from_centerline(centerline_points)

    # Create the path follower driver
    # Initial target speed is 5 m/s
    driver = veh.ChPathFollowerDriver(vehicle, path, "centerline", 5.0)

    # Configure steering controller
    driver.GetSteeringController().SetLookAheadDistance(5.0)  # 5 m lookahead
    driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)   # P=0.8, I=0, D=0

    # Configure speed controller
    driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)       # P=0.4, I=0, D=0

    driver.Initialize()

    return driver
