"""Reading/writing trajectory.csv files and interpolating trajectories.

The trajectory.csv contract is in docs/trajectory_contract.md:
- Columns defined by the last '# ' line before data (column names must be found by name, not position)
- Old 6-column files (t,x,y,yaw,v,yaw_rate) must still load
- 1 kHz sampling rate assumed
- Linear interpolation of all columns (yaw is unwrapped)
- loop=1: wrap at lap_time; loop=0: clamp
- If roll and pitch columns are absent, add sinusoidal body_motion from car_cfg (legacy)
"""
import io
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.integrate import odeint
from scipy.interpolate import interp1d

from .driver import CarState, CenterlineDriver


@dataclass
class TrajectoryData:
    """Parsed trajectory.csv data."""
    columns: dict  # column_name -> np.ndarray of values
    lap_time: float
    loop: int  # 1 = periodic, 0 = clamp
    source: str  # free text describing the source


def read_trajectory_csv(path) -> TrajectoryData:
    """Read trajectory.csv and return TrajectoryData.

    Columns are extracted from the last '# ' line before the data.
    Old 6-column files have columns='t,x,y,yaw,v,yaw_rate'.
    Raises ValueError if required columns are missing or if t is not monotonic.
    """
    path = Path(path)
    with open(path, 'r') as f:
        lines = f.readlines()

    # Parse header: extract lap_time, loop, source, and column names
    # First pass: collect all header lines
    header_lines = []
    for line in lines:
        if line.startswith("# "):
            header_lines.append(line[2:].strip())
        elif line.strip() == "" or line[0].isdigit() or line[0] == "-":
            break  # End of header

    lap_time = None
    loop = 1
    source = "unknown"
    columns_line = None

    # Find the column line (last line that looks like "t,x,y,...")
    for hline in reversed(header_lines):
        if "=" not in hline or hline.startswith("t,"):
            # This is the column line
            if hline.startswith("t,"):
                columns_line = hline
                break

    # Parse other header lines for metadata
    for hline in header_lines:
        if hline.startswith("t,"):
            # Skip the column line
            continue
        if "=" in hline:
            parts = hline.split("=", 1)
            if len(parts) == 2:
                key, val = parts[0].strip(), parts[1].strip()
                if key == "lap_time":
                    lap_time = float(val)
                elif key == "loop":
                    loop = int(val)
                elif key == "source":
                    source = val

    if columns_line is None:
        raise ValueError("No column names found in header (last '# ' line)")

    column_names = [c.strip() for c in columns_line.split(",")]

    # Check required columns
    required = {"t", "x", "y", "yaw"}
    if not required.issubset(set(column_names)):
        raise ValueError(f"Missing required columns {required - set(column_names)}")

    # Parse data: skip comment lines and empty lines, read numeric data
    data_lines = [l for l in lines if not l.startswith("#") and l.strip()]
    if not data_lines:
        raise ValueError("No data rows found")

    # Parse CSV data
    data = np.loadtxt(data_lines, delimiter=",", dtype=float)
    if data.ndim == 1:
        data = data[np.newaxis, :]

    if data.shape[1] != len(column_names):
        raise ValueError(
            f"Column count mismatch: {len(column_names)} column names but {data.shape[1]} data columns"
        )

    # Build columns dict
    columns = {name: data[:, i] for i, name in enumerate(column_names)}

    # Validate t is monotonic
    t = columns["t"]
    if not np.all(np.diff(t) > 0):
        raise ValueError("Time column 't' is not strictly monotonic")

    # Infer lap_time from last t if not in header
    if lap_time is None:
        lap_time = float(t[-1])

    return TrajectoryData(columns=columns, lap_time=lap_time, loop=loop, source=source)


def write_trajectory_csv(path, columns: dict, lap_time: float, loop: int = 1, source: str = "kinematic"):
    """Write trajectory.csv in the contract format.

    columns: dict of column_name -> np.ndarray
    Writes with %.9f precision and proper header.
    """
    path = Path(path)

    # Ensure we have at least t
    if "t" not in columns:
        raise ValueError("'t' column is required")

    # Ensure all columns have the same length
    n = len(columns["t"])
    for name, arr in columns.items():
        if len(arr) != n:
            raise ValueError(f"Column '{name}' has length {len(arr)}, expected {n}")

    # Build header
    header_lines = [
        f"# lap_time={lap_time:.9f}",
        f"# loop={loop}",
        f"# source={source}",
        f"# {','.join(columns.keys())}",
    ]

    # Stack columns in order
    col_order = list(columns.keys())
    data = np.column_stack([columns[name] for name in col_order])

    # Write with header
    with open(path, "w") as f:
        f.write("\n".join(header_lines) + "\n")
        np.savetxt(f, data, delimiter=",", fmt="%.9f")


class TrajectoryDriver:
    """Interpolate trajectory from CSV data or a CenterlineDriver.

    state_at(t) returns CarState with linear interpolation of all columns.
    If roll and pitch columns are missing, adds sinusoidal body_motion from car_cfg.
    """

    def __init__(self, csv_path_or_data, car_cfg=None):
        """Load trajectory from CSV path or TrajectoryData object.

        If car_cfg is provided and roll/pitch are missing, body_motion is added.
        """
        if isinstance(csv_path_or_data, (str, Path)):
            traj_data = read_trajectory_csv(csv_path_or_data)
        else:
            traj_data = csv_path_or_data

        self.columns = traj_data.columns
        self.lap_time = traj_data.lap_time
        self.loop = traj_data.loop
        self.source = traj_data.source
        self._t = self.columns["t"].copy()

        # Set up interpolation for each column
        # For yaw, we need to handle wrapping carefully during interpolation
        self._interps = {}
        for name, col in self.columns.items():
            # All columns use linear interpolation
            self._interps[name] = interp1d(
                self._t, col, kind="linear", bounds_error=False,
                fill_value=(col[0], col[-1])  # clamp at ends (will be handled by loop logic)
            )

        # Check if we need to add body_motion
        self._has_roll = "roll" in self.columns
        self._has_pitch = "pitch" in self.columns
        self._car_cfg = car_cfg

        if car_cfg and not self._has_roll and not self._has_pitch:
            # Add sinusoidal body_motion
            bm = car_cfg.get("body_motion", {}) or {}
            self._pa = math.radians(float(bm.get("pitch_amplitude_deg", 0.0)))
            self._ra = math.radians(float(bm.get("roll_amplitude_deg", 0.0)))
            self._fb = float(bm.get("frequency_hz", 1.5))
        else:
            self._pa = self._ra = self._fb = 0.0

    def state_at(self, t: float) -> CarState:
        """Interpolate car state at time t.

        Handles looping (loop=1) and clamping (loop=0).
        Returns CarState with all fields populated by interpolation or defaults.
        """
        # Handle time wrapping/clamping
        if self.loop == 1:
            t_eval = t % self.lap_time
        else:
            t_eval = np.clip(t, 0.0, self.lap_time)

        # Interpolate all columns
        result = {}
        for name in self.columns.keys():
            val = float(self._interps[name](t_eval))
            # Wrap yaw to [-pi, pi) on output
            if name == "yaw":
                val = (val + math.pi) % (2 * math.pi) - math.pi
            result[name] = val

        # Extract required fields
        x = result.get("x", 0.0)
        y = result.get("y", 0.0)
        yaw = result.get("yaw", 0.0)
        v = result.get("v", 0.0)
        yaw_rate = result.get("yaw_rate", 0.0)

        # Extract optional fields or use defaults
        z = result.get("z", 0.0)
        ax = result.get("ax", 0.0)
        ay = result.get("ay", 0.0)
        az = result.get("az", 9.81)
        wx = result.get("wx", 0.0)
        wy = result.get("wy", 0.0)
        wz = result.get("wz", yaw_rate)  # Default wz to yaw_rate per contract

        # Handle roll/pitch
        if self._has_roll and "roll" in result:
            roll = result["roll"]
        elif self._ra > 0:
            w = 2 * math.pi * self._fb
            roll = self._ra * math.sin(1.31 * w * t + 0.9)
        else:
            roll = 0.0

        if self._has_pitch and "pitch" in result:
            pitch = result["pitch"]
        elif self._pa > 0:
            w = 2 * math.pi * self._fb
            pitch = self._pa * math.sin(w * t)
        else:
            pitch = 0.0

        return CarState(
            x=x, y=y, yaw=yaw, roll=roll, pitch=pitch, v=v, yaw_rate=yaw_rate,
            z=z, ax=ax, ay=ay, az=az, wx=wx, wy=wy, wz=wz
        )


def kinematic_columns(
    driver: CenterlineDriver,
    rate_hz: float = 1000.0,
    tilt: Optional[dict] = None
) -> dict:
    """Generate trajectory columns for a full lap of the driver at the given rate.

    driver: CenterlineDriver or similar with state_at(t) -> CarState
    rate_hz: sample rate (default 1 kHz per contract)
    tilt: optional dict with keys: enabled, pitch_deg_per_g, roll_deg_per_g,
          natural_freq_hz, damping. If enabled=True, compute IMU-style accelerations
          and angular velocities using a tilt model.

    Returns:
        dict of column_name -> np.ndarray for one complete lap (last row ~= first row for periodic columns)

    Tilt Model (when enabled):
    -------------------------
    The car experiences:
    - Longitudinal acceleration: a_long = dv/dt
    - Lateral acceleration: a_lat = v * yaw_rate (points toward the turn center)

    Pitch model:
      target_pitch = pitch_deg_per_g * (-a_long / 9.81) deg
      Interpretation: braking (a_long < 0) -> target_pitch > 0 (nose down)
      This is filtered through a 2nd-order system to smooth the response.

    Roll model:
      target_roll = roll_deg_per_g * (a_lat / 9.81) deg
      Interpretation: left turn (yaw_rate > 0, a_lat > 0) -> body leans right (roll > 0)
      where roll > 0 means right-side-down per the contract.

    Body-frame IMU readings (specific force, gravity included):
      level-frame f = (a_long, a_lat, g), body-frame f_b = (Ry(pitch) @ Rx(roll))^T f
      so a level car reads (a_long, a_lat, ~9.81): az does not shrink under braking or cornering.
      wx = d(roll)/dt
      wy = d(pitch)/dt
      wz = yaw_rate (angular velocity about vertical axis)

    The roll/pitch filtering is made periodic-safe by running 3 laps and keeping the middle one.
    """
    lap_time = driver.lap_time
    n = int(np.round(lap_time * rate_hz)) + 1  # +1 for the closing row
    ts = np.linspace(0, lap_time, n)

    # Sample the driver
    states = [driver.state_at(t) for t in ts]

    # Extract base columns
    t = np.array([float(t) for t in ts])
    x = np.array([s.x for s in states])
    y = np.array([s.y for s in states])
    yaw = np.array([s.yaw for s in states])
    v = np.array([s.v for s in states])
    yaw_rate = np.array([s.yaw_rate for s in states])

    # Unwrap yaw for CSV (linear interpolation)
    yaw = np.unwrap(yaw)

    columns = {
        "t": t,
        "x": x,
        "y": y,
        "z": np.zeros_like(t),
        "yaw": yaw,
        "v": v,
        "yaw_rate": yaw_rate,
    }

    if tilt and tilt.get("enabled", False):
        # Compute tilt model
        pitch_deg_per_g = float(tilt.get("pitch_deg_per_g", 0.6))
        roll_deg_per_g = float(tilt.get("roll_deg_per_g", 1.0))
        nat_freq = float(tilt.get("natural_freq_hz", 2.5))
        damping = float(tilt.get("damping", 0.7))

        # Compute accelerations via finite differences
        # a_long = dv/dt
        a_long = np.gradient(v, t)  # Use numpy's gradient for better numerical stability
        # a_lat = v * yaw_rate
        a_lat = v * yaw_rate

        # Target angles in degrees
        target_pitch_deg = pitch_deg_per_g * (-a_long / 9.81)
        target_roll_deg = roll_deg_per_g * (a_lat / 9.81)

        # Convert to radians
        target_pitch = np.radians(target_pitch_deg)
        target_roll = np.radians(target_roll_deg)

        # Filter pitch and roll through a 2nd-order system
        # To make it periodic-safe, run 3 laps and keep the middle one
        ts_3lap = np.linspace(0, 3 * lap_time, 3 * n)
        states_3lap = [driver.state_at(float(t)) for t in ts_3lap]
        v_3lap = np.array([s.v for s in states_3lap])
        yaw_rate_3lap = np.array([s.yaw_rate for s in states_3lap])

        a_long_3lap = np.gradient(v_3lap, ts_3lap)
        a_lat_3lap = v_3lap * yaw_rate_3lap

        target_pitch_deg_3lap = pitch_deg_per_g * (-a_long_3lap / 9.81)
        target_roll_deg_3lap = roll_deg_per_g * (a_lat_3lap / 9.81)

        target_pitch_3lap = np.radians(target_pitch_deg_3lap)
        target_roll_3lap = np.radians(target_roll_deg_3lap)

        # Second-order filter: x'' + 2*zeta*wn*x' + wn^2*x = wn^2*u
        # State: [pitch/roll, pitch_rate/roll_rate]
        wn = 2 * math.pi * nat_freq

        def second_order_system(y, t_arr, target, wn, zeta):
            """Integrate 2nd-order system to track target."""
            n = len(t_arr)
            y_out = np.zeros(n)
            dy_out = np.zeros(n)
            y_out[0] = target[0]
            dy_out[0] = 0.0

            for i in range(1, n):
                dt = t_arr[i] - t_arr[i - 1]
                # Simple forward Euler: y' = v, v' = wn^2*(target - y) - 2*zeta*wn*v
                v = dy_out[i - 1]
                a = wn * wn * (target[i] - y_out[i - 1]) - 2 * zeta * wn * v
                y_out[i] = y_out[i - 1] + v * dt
                dy_out[i] = v + a * dt

            return y_out, dy_out

        pitch_3lap, pitch_rate_3lap = second_order_system(
            np.zeros_like(ts_3lap), ts_3lap, target_pitch_3lap, wn, damping
        )
        roll_3lap, roll_rate_3lap = second_order_system(
            np.zeros_like(ts_3lap), ts_3lap, target_roll_3lap, wn, damping
        )

        # Extract middle lap
        pitch = pitch_3lap[n : 2 * n]
        roll = roll_3lap[n : 2 * n]
        pitch_rate = pitch_rate_3lap[n : 2 * n]
        roll_rate = roll_rate_3lap[n : 2 * n]

        # IMU specific force in the body frame (gravity included).
        # In the heading ("level") frame the specific force is f = (a_long, a_lat, g): the car's
        # acceleration plus the upward reaction to gravity. a_lat > 0 points left (towards the
        # turn centre). The body is tilted by R = Ry(pitch) @ Rx(roll) relative to that frame, so
        # f_body = R^T f. On flat ground az stays ~g (it does NOT shrink in corners or under braking).
        g = 9.81
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        fx, fy, fz = a_long, a_lat, g
        ax = cp * fx - sp * fz
        ay = sp * sr * fx + cr * fy + cp * sr * fz
        az = sp * cr * fx - sr * fy + cp * cr * fz

        # Angular velocities in body frame
        # wz = yaw_rate (rotation about vertical)
        # wx, wy = time derivatives of roll, pitch
        wx = np.gradient(roll, t)
        wy = np.gradient(pitch, t)
        wz = yaw_rate

        columns["roll"] = roll
        columns["pitch"] = pitch
        columns["ax"] = ax
        columns["ay"] = ay
        columns["az"] = az
        columns["wx"] = wx
        columns["wy"] = wy
        columns["wz"] = wz
    else:
        # No tilt model: provide defaults
        columns["roll"] = np.zeros_like(t)
        columns["pitch"] = np.zeros_like(t)
        columns["ax"] = np.zeros_like(t)
        columns["ay"] = np.zeros_like(t)
        columns["az"] = 9.81 * np.ones_like(t)
        columns["wx"] = np.zeros_like(t)
        columns["wy"] = np.zeros_like(t)
        columns["wz"] = yaw_rate.copy()

    return columns
