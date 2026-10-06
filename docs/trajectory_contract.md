# trajectory.csv contract

One file describes how the car moves. It is produced by one of:
- `scripts/export_bundle.py` from the kinematic `CenterlineDriver` (`source=kinematic`),
- the Chrono vehicle-dynamics run in `dynamics/` (`source=chrono`),

and consumed by the C++ renderer (`cvsim::Trajectory`) and the Python `TrajectoryDriver`
(so the Python golden frames match the C++ output).

## Format
UTF-8 text. Lines starting with `# ` are header comments. Then comma-separated numeric rows,
sampled uniformly at 1 kHz starting at t = 0.

```
# lap_time=37.446782616        duration covered by the rows in seconds (name kept for backward compat)
# loop=1                       1 = periodic lap (wrap around), 0 = single run (clamp at both ends). Default 1 if absent.
# source=chrono                free text
# <free text comment lines>
# t,x,y,z,roll,pitch,yaw,v,yaw_rate,ax,ay,az,wx,wy,wz       <- LAST '# ' line before the data = column names
0.000000000,0.0,...
```

Readers must find columns BY NAME from that last `# ` line (the old 6-column files have
`# t,x,y,yaw,v,yaw_rate`).

## Columns
| name | unit | required | meaning | default if missing |
|---|---|---|---|---|
| t | s | yes | time, uniform 1 kHz from 0 | |
| x, y | m | yes | position of `base_link` in the world frame (x fwd at start, y left, z up) | |
| yaw | rad | yes | heading, counter-clockwise positive, **unwrapped** (continuous) | |
| z | m | no | vertical offset of the base_link ground point from the flat ground (bounce); 0 at rest | 0 |
| roll | rad | no | rotation about the car's +x axis. Positive = **right side down** | see below |
| pitch | rad | no | rotation about the car's +y axis. Positive = **nose down** | see below |
| v | m/s | no | forward speed | 0 |
| yaw_rate | rad/s | no | | 0 |
| ax, ay, az | m/s^2 | no | what an IMU measures: **specific force in the body frame** (x fwd, y left, z up), gravity INCLUDED, so a level stationary car reads az = +9.81 | 0, 0, 9.81 |
| wx, wy, wz | rad/s | no | angular velocity in the body frame | 0, 0, yaw_rate |

Orientation is `R = Rz(yaw) * Ry(pitch) * Rx(roll)` (same as `render/sim.py euler_to_quat`).
Roll and pitch are **relative to the at-rest pose**: a stationary car on flat ground reads 0, 0.

## Roll/pitch missing?
If the file has NO `roll` and NO `pitch` column, the renderer adds the sinusoidal
`car.yaml body_motion` (legacy behaviour). If either column exists, **nothing is added**: the CSV is the truth.

## Interpolation (all readers must agree)
- Linear interpolation of every column between rows (yaw is unwrapped so linear is fine; wrap it to [-pi, pi) on output).
- `loop=1`: `t' = t mod lap_time`; the last row is expected to equal the first row (with yaw + n*2*pi), so
  interpolation between the last row and the first row wraps seamlessly.
- `loop=0`: `t` is clamped to `[0, last t]`.

## Frames
World frame = the track frame of `track.yaml` (the MuJoCo world): x/y on the ground, z up.
Chrono's ISO frame (x fwd, y left, z up) is already the same convention.
