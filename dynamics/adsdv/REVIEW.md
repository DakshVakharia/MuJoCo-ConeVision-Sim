# Review of the researched parameters (read this before using adsdv_params.yaml)

`adsdv_params.yaml` / `PARAMETERS.md` were written by a research agent. Most numbers are **assumptions**
for a generic Formula-Student-class car. The **ADS-DV's own mass, inertia, spring and damper values are
not public**, so this is NOT a model of the real ADS-DV. Call it "FS-class car" in results.

Solidly sourced: wheelbase 1.55 m and the 365 kg / Cd 0.4 / A 1.0 m^2 figures come from one FS paper
(doi 10.3390/vehicles8050111). Everything else is typical-value guesswork.

## Problems found and the corrected values to use

| Parameter | In the YAML | Problem | Use instead |
|---|---|---|---|
| tyre unloaded radius | 0.33 m | road-car size; 13" rim diameter (330 mm) was mistaken for tyre radius; our MuJoCo car uses 0.23 m | **0.23 m** (consistent with `config/car.yaml`) |
| inertia labels | `inertia_pitch_ixx`=50, `inertia_roll_iyy`=30 | pitch is about y (Iyy) and roll about x (Ixx): names are swapped | **Ixx (roll) ~35, Iyy (pitch) ~90, Izz (yaw) ~100 kg m^2** (assumption: m*k^2 with k ~ 0.55-0.6 m for pitch/yaw) |
| damper rate | 5000-6000 Ns/m | with 50 kN/m springs and ~62 kg sprung mass per corner: zeta = c/(2 sqrt(k m)) = 5000/3535 = 1.4 (heavily over-damped) | **~2000-2500 Ns/m** (zeta ~ 0.6) |
| spring rate | 50 kN/m | ride frequency = sqrt(k/m)/2pi = 4.5 Hz; the YAML's own target is 2-3 Hz (FS cars are typically ~2.5-3.5 Hz) | **~25-35 kN/m wheel rate** (check units: Chrono's JSON spring acts along the strut, so account for the motion ratio) |
| brake torque | 1200 Nm per wheel | 1.5 g stop of a 320 kg car needs ~1200 N per wheel x 0.25 m radius = ~300 Nm; 1200 Nm just locks the wheels | **~350 Nm per wheel** |
| mass | 320 kg "midpoint of 255 and 365" | the midpoint is 310, and no source is given for 320 | keep **~320 kg incl. driver** but label it assumption |

## Expected body motion (use these to sanity-check the Chrono result)

Back-of-envelope with m = 320 kg, h_cg = 0.30 m, wheelbase 1.55 m, track 1.2 m, wheel rate 50 kN/m
(the original spring rate; halving it roughly doubles the numbers below):
- roll stiffness ~ 2 * k * (t/2)^2 * 2 axles = 72 kNm/rad (+ anti-roll bars) -> roll moment per g
  ~ 250 kg * 9.81 * (0.30 - 0.05) = 613 Nm -> **~0.4 deg per g** (stiff FS cars: 0.4-1.5 deg per g)
- pitch stiffness ~ 120 kNm/rad -> braking moment per g ~ 320 * 9.81 * 0.30 = 942 Nm -> **~0.45 deg per g**
  (less with anti-dive)
So a plausible result for this car is roughly 0.3-1.5 deg per g in both axes. If the Chrono run gives
something far outside that range (e.g. 5 deg per g or 0.01 deg per g), a parameter is wrong.
Braking at ~1 g therefore tilts the camera by only about half a degree: small, but for distance it
matters a lot at long range. For a camera h = 1.15 m above the ground, the ground distance of a cone
base is d = h / tan(angle below horizon), so a pitch error of d_theta shifts it by about d^2 / h * d_theta.
With d_theta = 0.5 deg = 0.0087 rad: ~0.8 m at 10 m range, ~3 m at 20 m, ~7 m at 30 m.
This is exactly why the depth pipeline's IMU tilt correction matters, and why this sim is useful.

## Which stock Chrono vehicle to start from
`C:\Users\daksh\miniforge3\envs\chrono\Library\data\vehicle\sedan\` (double wishbone). See
`CHRONO_MAPPING.md` for the JSON keys; apply the corrected values above, not the originals.
