# Chrono Vehicle JSON Mapping for FS-AI ADS-DV

## Stock Vehicle Selection & Reasoning

**Recommended Base Vehicle: Sedan**
- **Location**: `C:\Users\daksh\miniforge3\envs\chrono\Library\data\vehicle\sedan\`
- **Why Sedan**: 
  - Double-wishbone suspension (matches Formula Student standard)
  - Smaller than `generic` (sedan chassis 1550 kg vs. generic 2086 kg; sedan wheelbase 2.776 m vs. generic 2.5 m; sedan has more manageable spring/damper values)
  - Has complete set of JSON templates for all subsystems (chassis, suspension F/R, steering, wheels, tires, brakes, driveline)
  - Use Sedan as a scaling reference; our target ADS-DV is much smaller (~320 kg vs. 1550 kg chassis)
- **Alternative considered**: Generic (too heavy), HMMWV (military truck, wrong suspension), Duro (cargo truck)

---

## Chrono Vehicle Data Directory Structure

```
C:\Users\daksh\miniforge3\envs\chrono\Library\data\vehicle\sedan\
├── Sedan.json                                    # Top-level vehicle assembly
├── chassis/
│   └── Sedan_Chassis.json                        # Chassis mass, inertia, driver position
├── suspension/
│   ├── Sedan_DoubleWishbone.json                 # Front/rear suspension geometry, springs, shocks
│   └── Sedan_MultiLink.json                      # Alternative (not using)
├── wheel/
│   └── Sedan_Wheel.json                          # Wheel mass, inertia, visualization
├── tire/
│   ├── Sedan_TMeasyTire.json                     # Tire model (empirical, recommended)
│   ├── Sedan_Pac02Tire.json                      # Magic Formula tire (requires .tir file)
│   └── Sedan_RigidTire.json                      # Rigid tire (not realistic)
├── steering/
│   └── Sedan_RackPinion.json                     # Steering linkage, max angle
├── brake/
│   ├── Sedan_BrakeSimple.json                    # Simple max-torque brake
│   └── Sedan_BrakeShafts.json                    # Alternative (not using)
├── driveline/
│   └── Sedan_Driveline2WD.json                   # Gear ratio, differential, shaft inertia
├── powertrain/
│   ├── Sedan_EngineSimpleMap.json                # Engine torque map (RPM-based)
│   ├── Sedan_EngineShafts.json                   # Detailed engine model
│   ├── Sedan_AutomaticTransmissionSimpleMap.json # Automatic transmission
│   └── Sedan_ManualTransmissionShafts.json       # Manual transmission
└── driver/
    └── Sedan_AIDriver.json                       # AI driver parameters (optional)
```

---

## JSON File Structure & Parameter Mapping

### 1. **Top-Level Vehicle Assembly** (`Sedan.json`)

This is the entry point that glues everything together.

**File Structure:**
```json
{
  "Name": "Sedan",
  "Type": "Vehicle",
  "Template": "WheeledVehicle",
  "Vehicle": { "Input File": "sedan/vehicle/Sedan_Vehicle.json", ... },
  "Powertrain": { "Input File": "sedan/powertrain/Sedan_SimpleMapPowertrain.json" },
  "Tire": { "Input File": "sedan/tire/Sedan_TMeasyTire.json", ... },
  "Zombie": { ... }  // Visualization properties
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | Notes |
|---------------|-------------|----------|-------|
| — | `Sedan.json` | `"Input File"` under `"Vehicle"` | Points to `sedan/vehicle/Sedan_Vehicle.json` (defines axles, suspension, steering) |
| — | `Sedan.json` | `"Input File"` under `"Powertrain"` | Points to powertrain JSON (engine/transmission model) |
| — | `Sedan.json` | `"Input File"` under `"Tire"` | Points to tire JSON (e.g., `sedan/tire/Sedan_TMeasyTire.json`) |

---

### 2. **Vehicle Geometry & Axle Layout** (`sedan/vehicle/Sedan_Vehicle.json`)

Defines wheelbase, axle positions, and suspension mounting points.

**File Structure:**
```json
{
  "Chassis": { "Input File": "sedan/chassis/Sedan_Chassis.json" },
  "Axles": [
    {
      "Suspension Input File": "sedan/suspension/Sedan_DoubleWishbone.json",
      "Suspension Location": [1.388, 0, 0.25],
      "Steering Index": 0,
      "Left Wheel Input File": "sedan/wheel/Sedan_Wheel.json",
      "Left Brake Input File": "sedan/brake/Sedan_BrakeSimple.json"
    },
    { /* Rear axle */ }
  ],
  "Steering Subsystems": [...],
  "Wheelbase": 2.776,
  "Maximum Steering Angle (deg)": 25.0,
  "Driveline": { "Input File": "sedan/driveline/Sedan_Driveline2WD.json", ... }
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| `wheelbase_m` = 1.550 | `Sedan_Vehicle.json` | `"Wheelbase"` | Change from 2.776 → 1.550 |
| Axle locations | `Sedan_Vehicle.json` | `"Suspension Location"` under each axle | Scale from sedan proportions |
| `maximum_steering_angle_deg` = 20 | `Sedan_Vehicle.json` | `"Maximum Steering Angle (deg)"` | Change from 25.0 → 20.0 |
| Track widths | Derived from suspension geometry | Suspension XML hardpoints | Adjust via suspension control arm positions |

---

### 3. **Chassis & Mass Properties** (`sedan/chassis/Sedan_Chassis.json`)

Defines the chassis body mass, inertia, and center of gravity location.

**File Structure:**
```json
{
  "Name": "Sedan chassis",
  "Type": "Chassis",
  "Template": "RigidChassis",
  "Components": [
    {
      "Centroidal Frame": {
        "Location": [0, 0, 0.7],           // CoG location in vehicle frame
        "Orientation": [1, 0, 0, 0]         // Quaternion (identity)
      },
      "Mass": 1550,                         // kg
      "Moments of Inertia": [222.8, 944.1, 1053.5],  // [Ixx, Iyy, Izz] in kg·m²
      "Products of Inertia": [0, 0, 0]      // Off-diagonal terms
    }
  ],
  "Driver Position": { "Location": [...] },
  "Rear Connector Location": [...]
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| `cog_height_m` = 0.30 | `Sedan_Chassis.json` | `"Centroidal Frame"."Location"[2]` (z-component) | Change from 0.7 → 0.30 |
| `cog_longitudinal_from_front_axle_m` = 0.75 | `Sedan_Chassis.json` | `"Centroidal Frame"."Location"[0]` (x-component) | Requires calculation: adjust front axle position in Sedan_Vehicle.json; CoG x is vehicle frame origin |
| `total_mass_kg` = 320 | `Sedan_Chassis.json` | `"Mass"` | Change from 1550 → 320 |
| `inertia_pitch_ixx_kg_m2` = 50 | `Sedan_Chassis.json` | `"Moments of Inertia"[1]` (Iyy/pitch) | Change from 944.1 → 50 |
| `inertia_roll_iyy_kg_m2` = 30 | `Sedan_Chassis.json` | `"Moments of Inertia"[0]` (Ixx/roll) | Change from 222.8 → 30 |
| `inertia_yaw_izz_kg_m2` = 60 | `Sedan_Chassis.json` | `"Moments of Inertia"[2]` (Izz/yaw) | Change from 1053.5 → 60 |

**Note:** In Chrono's convention, the inertia tensor at the centroidal frame is:
- **Ixx** (roll/rotation about x-axis / longitudinal) = typically ~30% of Iyy for cars
- **Iyy** (pitch/rotation about y-axis / lateral) = typically largest for long wheelbase
- **Izz** (yaw/rotation about z-axis / vertical) = typically ~1.2× Iyy

For FS car (short wheelbase): scale all by (1.550/2.776)² ≈ 0.31 of sedan values.

---

### 4. **Suspension - Front & Rear** (`sedan/suspension/Sedan_DoubleWishbone.json`)

Defines suspension geometry, spring rates, damping coefficients, and hardpoint locations.

**File Structure (excerpt):**
```json
{
  "Name": "Sedan DoubleWishbone",
  "Type": "Suspension",
  "Template": "DoubleWishbone",
  "Camber Angle (deg)": 0,
  "Toe Angle (deg)": 0,
  
  "Spindle": { "Mass": 1.103, "COM": [...], "Inertia": [...] },
  "Upright": { "Mass": 1.397, "COM": [...], "Moments of Inertia": [...] },
  "Upper Control Arm": { "Mass": 1.032, "COM": [...], ..., "Location Chassis Front": [...], "Location Chassis Back": [...], "Location Upright": [...] },
  "Lower Control Arm": { ... },
  
  "Spring": {
    "Location Chassis": [...],
    "Location Arm": [...],
    "Free Length": 0.51,
    "Spring Coefficient": 500000.0      // N/m
  },
  
  "Shock": {
    "Location Chassis": [...],
    "Location Arm": [...],
    "Damping Coefficient": 10000.0      // Ns/m
  },
  
  "Axle": { "Inertia": 0.4 }
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| `spring_rate_front_n_per_m` = 50000 | `Sedan_DoubleWishbone.json` (front) | `"Spring"."Spring Coefficient"` | Change from 500000.0 → 50000 |
| `spring_rate_rear_n_per_m` = 50000 | `Sedan_DoubleWishbone.json` (rear) | `"Spring"."Spring Coefficient"` | Change from 500000.0 → 50000 |
| `damper_rate_front_low_speed_ns_per_m` = 5000 | `Sedan_DoubleWishbone.json` (front) | `"Shock"."Damping Coefficient"` | Change from 10000.0 → 5000 |
| `damper_rate_rear_low_speed_ns_per_m` = 6000 | `Sedan_DoubleWishbone.json` (rear) | `"Shock"."Damping Coefficient"` | Change from 10000.0 → 6000 |
| `camber_angle_front_deg` = 0 | `Sedan_DoubleWishbone.json` (front) | `"Camber Angle (deg)"` | Change from 0 → 0 (no change for baseline) |
| `toe_angle_front_deg` = 0 | `Sedan_DoubleWishbone.json` (front) | `"Toe Angle (deg)"` | Change from 0 → 0 (no change for baseline) |
| `anti_roll_bar_stiffness_front_nm_per_rad` = 8000 | `Sedan_DoubleWishbone.json` (front) | (Not directly in JSON; need to add or leave hardcoded in C++ if Chrono supports it) | Advanced: may require Chrono C++ configuration |
| Suspension hardpoint positions (track width, roll center height) | `Sedan_DoubleWishbone.json` | `"Upper Control Arm"."Location Chassis Front/Back"`, `"Location Upright"`, etc. | Adjust control arm geometry to achieve target track width (1200 mm) and roll center height |

**Note on Hardpoints:**
- `"Location Chassis Front"` and `"Location Chassis Back"`: mounting points on chassis (vehicle frame origin at CoG)
- `"Location Upright"`: mounting point on wheel upright (must be modified to scale track width)
- For FS car with 1200 mm track, uprights should be ±0.6 m from centerline (vs. sedan's smaller track)

---

### 5. **Wheel** (`sedan/wheel/Sedan_Wheel.json`)

Defines wheel mass and inertia (not tire; tire is separate).

**File Structure:**
```json
{
  "Name": "Sedan Wheel",
  "Type": "Wheel",
  "Template": "Wheel",
  "Mass": 13.2,                           // kg
  "Inertia": [0.24, 0.42, 0.24],          // kg·m²
  "Visualization": { ... }
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| `wheel_mass_per_wheel_kg` = 6.0 | `Sedan_Wheel.json` | `"Mass"` | Change from 13.2 → 6.0 |
| Wheel inertia | `Sedan_Wheel.json` | `"Inertia"` | Scale by (R_FS/R_sed)² if radius changes; typical ratio 0.24/0.42/0.24 for [Ixx, Iyy, Izz] |

---

### 6. **Tire** (`sedan/tire/Sedan_TMeasyTire.json`)

Defines tire mechanical properties (empirical TMeasy model is easiest to edit).

**File Structure (TMeasy - recommended for FS):**
```json
{
  "Name": "Sedan TMeasy Tire",
  "Type": "Tire",
  "Template": "TMeasyTire",
  
  "Design": {
    "Unloaded Radius [m]": 0.3266,
    "Mass [kg]": 11.5,
    "Inertia [kg.m2]": [0.156, 0.679, 0.156],
    "Width [m]": 0.245,
    "Rim Radius [m]": 0.2286
  },
  
  "Coefficient of Friction": 0.8,
  "Rolling Resistance Coefficient": 0.015,
  "Vehicle Type": "Passenger",
  "Load Index": 97,
  "Maximum Bearing Capacity [N]": 10000
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| `tire_unloaded_radius_m` = 0.330 | `Sedan_TMeasyTire.json` | `"Design"."Unloaded Radius [m]"` | Change from 0.3266 → 0.330 (13" tire) |
| `tire_mass_per_wheel_kg` = 6.0 | `Sedan_TMeasyTire.json` | `"Design"."Mass [kg]"` | Change from 11.5 → 6.0 |
| `tire_width_m` = 0.200 | `Sedan_TMeasyTire.json` | `"Design"."Width [m]"` | Change from 0.245 → 0.200 |
| Wheel rim diameter (13" = 330 mm) | `Sedan_TMeasyTire.json` | `"Design"."Rim Radius [m]"` | Change from 0.2286 → 0.1651 (13" rim = 330 mm diameter = 165.1 mm radius) |
| `tire_friction_coefficient` = 0.80 | `Sedan_TMeasyTire.json` | `"Coefficient of Friction"` | Change from 0.8 → 0.80 (no change, already correct) |
| `rolling_resistance_coefficient` = 0.015 | `Sedan_TMeasyTire.json` | `"Rolling Resistance Coefficient"` | Change from 0.015 → 0.015 (no change) |
| `tire_vertical_stiffness_n_per_m` = 150000 | `Sedan_TMeasyTire.json` | `"Maximum Bearing Capacity [N]"` / tire deformation | TMeasy does NOT expose vertical stiffness directly; it is implicit in the load-deformation curve. Approximate: max bearing capacity ~150 kN typical for racing tire |

**Note on Tire Stiffness:** TMeasy tire model uses empirical curves and does not expose vertical spring stiffness. To add vertical stiffness tuning, consider using the **Pac02Tire** (Magic Formula) model instead, which requires a `.tir` file with detailed Pacejka coefficients (not included in this repo).

---

### 7. **Steering** (`sedan/steering/Sedan_RackPinion.json`)

Defines steering linkage and maximum steering angle.

**File Structure:**
```json
{
  "Name": "Sedan Rack-Pinion Steering",
  "Type": "Steering",
  "Template": "RackPinion",
  
  "Steering Link": {
    "Mass": 1.889,
    "COM": 0,
    "Inertia": [0.138, 0.00009, 0.138],
    "Radius": 0.03,
    "Length": 0.5
  },
  
  "Pinion": {
    "Radius": 0.3,
    "Maximum Angle (deg)": 15.5
  }
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| Steering link mass | `Sedan_RackPinion.json` | `"Steering Link"."Mass"` | Scale by mass ratio; e.g., 320/1550 = 0.206 → 1.889 × 0.206 ≈ 0.39 kg |
| `maximum_steering_angle_deg` = 20 | `Sedan_RackPinion.json` | `"Pinion"."Maximum Angle (deg)"` | Change from 15.5 → 20 (or match vehicle-level setting) |
| Pinion radius (steering ratio) | `Sedan_RackPinion.json` | `"Pinion"."Radius"` | Adjust to match steering ratio; 0.3 m is a reference value |

---

### 8. **Brake** (`sedan/brake/Sedan_BrakeSimple.json`)

Defines maximum brake torque (simple model; sufficient for dynamics).

**File Structure:**
```json
{
  "Name": "Sedan Brake",
  "Type": "Brake",
  "Template": "BrakeSimple",
  "Maximum Torque": 2000        // Nm per wheel
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| `maximum_brake_torque_per_wheel_nm` = 1200 | `Sedan_BrakeSimple.json` | `"Maximum Torque"` | Change from 2000 → 1200 (scaled by mass ratio: 2000 × 320/1550 ≈ 413, but use 1200 for small car typical) |

**Note:** Chrono's simple brake model applies max torque when commanded. For more detail (hydraulic lines, brake fade, distribution), see `Sedan_BrakeShafts.json` (more complex).

---

### 9. **Driveline** (`sedan/driveline/Sedan_Driveline2WD.json`)

Defines gear ratios, differential behavior, and driveshaft inertia.

**File Structure:**
```json
{
  "Name": "Sedan RWD Driveline",
  "Type": "Driveline",
  "Template": "ShaftsDriveline2WD",
  
  "Shaft Direction": {
    "Motor Block": [1, 0, 0],
    "Axle": [0, 1, 0]
  },
  
  "Shaft Inertia": {
    "Driveshaft": 0.5,
    "Differential Box": 0.6
  },
  
  "Gear Ratio": {
    "Conical Gear": 0.2
  },
  
  "Axle Differential Locking Limit": 100
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| `gear_ratio_final_drive` = 0.20 | `Sedan_Driveline2WD.json` | `"Gear Ratio"."Conical Gear"` | Change from 0.2 → 0.20 (no change for baseline FS; tunable for speed/torque tradeoff) |
| Driveshaft inertia | `Sedan_Driveline2WD.json` | `"Shaft Inertia"."Driveshaft"` | Scale by mass ratio; e.g., 0.5 × √(320/1550) ≈ 0.2 kg·m² |
| Differential type (open/locking) | `Sedan_Driveline2WD.json` | `"Axle Differential Locking Limit"` | 100 = open differential (no locking); lower values increase locking friction |

---

### 10. **Powertrain** (`sedan/powertrain/Sedan_EngineSimpleMap.json`)

Defines engine torque map (for FS, likely replaced with electric motor parameters).

**File Structure (Combustion Engine Example):**
```json
{
  "Name": "Sedan Simple Map Engine",
  "Type": "Engine",
  "Template": "EngineSimpleMap",
  
  "Maximal Engine Speed RPM": 6500,
  "Map Full Throttle": [
    [-10.0, 104.64],
    [1000, 236.8],
    [2000, 370.0],
    ...
    [6500, 244.6]
  ],
  "Map Zero Throttle": [...]
}
```

**Mapping Table:**

| Our Parameter | Chrono File | JSON Key | What to Change |
|---------------|-------------|----------|-----------------|
| `motor_max_power_kw` = 60 | Powertrain JSON (varies by template) | Depends on engine/motor template | For FS electric: replace with `MotorSimple` or create torque map at 0–500 RPM motor speed |
| `motor_max_torque_nm` = 150 | Powertrain JSON | Engine/motor torque output at different speeds | Adjust torque curve to match motor spec |

**Note:** For an electric FS vehicle, replace `EngineSimpleMap.json` with a simpler electric motor model. Chrono may provide `MotorDC.json` or similar; create a constant-torque curve up to max RPM.

---

## Summary: Copy-and-Edit Workflow

1. **Copy the Sedan folder** to `dynamics/adsdv/chrono_json/`:
   ```bash
   cp -r "C:\Users\daksh\miniforge3\envs\chrono\Library\data\vehicle\sedan" "C:\Users\daksh\FS-Monocular-Camera-Sim\dynamics\adsdv\chrono_json\adsdv"
   ```

2. **Edit JSON files** in order of precedence:
   - **Priority 1** (most critical for pitch/roll): `chassis/Sedan_Chassis.json` (mass, inertia, CoG height)
   - **Priority 2** (spring-mass natural frequency): `suspension/Sedan_DoubleWishbone.json` (spring rate, damping)
   - **Priority 3** (geometry): `vehicle/Sedan_Vehicle.json` (wheelbase, axle positions), suspension hardpoints for track width
   - **Priority 4** (minor effect): `wheel/Sedan_Wheel.json`, `tire/Sedan_TMeasyTire.json`, `brake/Sedan_BrakeSimple.json`
   - **Priority 5** (performance, not dynamics): driveline, powertrain, steering

3. **Validate** by loading the model in Python PyChrono and checking:
   - CoG height (z in chassis)
   - Mass (kg)
   - Inertias (kg·m²)
   - Spring natural frequency: f = sqrt(k / m_eff) / (2π)
   - Target pitch/roll freq ~2–3 Hz for FS

---

## Key JSON Field Reference (Quick Lookup)

| Subsystem | File | Field Name | Data Type | Units | Example |
|-----------|------|------------|-----------|-------|---------|
| **Chassis** | Chassis.json | `"Mass"` | float | kg | 1550 |
| | | `"Moments of Inertia"` | [3] floats | kg·m² | [222.8, 944.1, 1053.5] |
| | | `"Centroidal Frame"."Location"` | [3] floats | m | [0, 0, 0.7] |
| **Suspension** | DoubleWishbone.json | `"Spring Coefficient"` | float | N/m | 500000.0 |
| | | `"Damping Coefficient"` | float | Ns/m | 10000.0 |
| | | `"Camber Angle (deg)"` | float | deg | 0 |
| | | Control Arm `"Location Chassis Front"` | [3] floats | m | [-0.100, 0.4700, 0.1050] |
| **Wheel** | Wheel.json | `"Mass"` | float | kg | 13.2 |
| | | `"Inertia"` | [3] floats | kg·m² | [0.24, 0.42, 0.24] |
| **Tire** | TMeasyTire.json | `"Unloaded Radius [m]"` | float | m | 0.3266 |
| | | `"Mass [kg]"` | float | kg | 11.5 |
| | | `"Coefficient of Friction"` | float | dimensionless | 0.8 |
| **Vehicle** | Sedan_Vehicle.json | `"Wheelbase"` | float | m | 2.776 |
| | | `"Maximum Steering Angle (deg)"` | float | deg | 25.0 |
| | | Axle `"Suspension Location"` | [3] floats | m | [1.388, 0, 0.25] |
| **Brake** | BrakeSimple.json | `"Maximum Torque"` | float | Nm | 2000 |
| **Driveline** | Driveline2WD.json | `"Conical Gear"` | float | (ratio) | 0.2 |

---

## Notes & Gotchas

1. **Coordinate System**: Chrono uses a vehicle-frame origin typically at the rear axle centerline (or front, depending on setup). Sedan uses CoG as centroidal frame origin. Check the offset when adjusting axle positions.

2. **Scaling Inertias**: When scaling from sedan (wheelbase 2.776 m) to FS (1.550 m):
   - Linear dimensions: scale by 1.550/2.776 = 0.559
   - Inertias: scale by (0.559)² × (mass ratio) = 0.312 × (M_FS / M_sedan)

3. **Spring-Damper Tuning**: After editing JSON, simulate and check:
   - Pitch natural frequency: ~2–3 Hz for FS cars
   - Damping ratio: ~0.6–0.8 (0.6 = light, 0.8 = sluggish)
   - Use: ζ = C / (2 × sqrt(k × m))

4. **Anti-Roll Bar**: TMeasy suspension JSON does NOT directly expose ARB stiffness. If needed, either:
   - Adjust individual spring rates (front and rear asymmetry)
   - Use Chrono's C++ API to add ARB constraints
   - Increase damping to simulate ARB effect

5. **Tire Model**: TMeasy is empirical and fast. Pac02 (Magic Formula) requires external `.tir` file. For camera pitch/roll, vertical stiffness is in TMeasy's load-deformation curve (not user-editable in JSON).

6. **Driver Mass**: Sedan_Chassis.json has `"Driver Position"`. For CoG calculation, add 70 kg driver mass if needed; recalculate CoG: CoG_vehicle = (M_chassis × CoG_chassis + M_driver × CoG_driver) / (M_chassis + M_driver).
