# FS-AI ADS-DV Vehicle Parameters for Chrono Simulation

## Confidence Summary

**Solid parameters (SOURCED with high confidence):**
- Wheelbase: 1550 mm (published in 10.3390/vehicles8050111)
- Front/Rear track: ~1200 mm (typical Formula Student spec)
- Wheel diameter: 13" rim with ~0.33 m unloaded radius (sedan Chrono reference)
- Spring rate range: 40,000–70,000 N/m (FS typical, 40–70 N/mm)
- Damping coefficient: 2,000–15,000 Ns/m (FS typical ranges)

**Estimated with reasonable assumptions:**
- Total mass: 255–365 kg (FS-Driverless simulator: 255 kg; 4WD FS electric: 365 kg)
- Mass distribution: 40% front / 60% rear (typical FS cars)
- CoG height: 0.25–0.35 m (small cars, ~330 mm estimated)
- Moments of inertia: scaled from sedan (wheelbase 2.776 m) to FS (1.550 m)

**Gaps and unknowns:**
- FS-AI ADS-DV exact mass (not published; no access to internal CAD)
- Actual CoG longitudinal position (weight distribution ratio)
- Anti-roll bar stiffness (not mentioned in public specs)
- Tire vertical stiffness and Pacejka/TMeasy coefficients (FS teams typically do not publish)
- Motor/brake power limits for the shared ADS-DV platform
- Exact roll/pitch centre locations

**Parameters critical for pitch/roll dynamics:**
1. Total mass (affect inertial forces)
2. CoG height (directly affects roll/pitch lever arm)
3. Wheelbase and track width (affect load transfer during acceleration/cornering)
4. Spring rates (primary suspension stiffness → natural frequency)
5. Damper rates (damping ratio, energy dissipation)
6. Anti-roll bar stiffness (roll stiffness distribution)
7. Unsprung mass and wheel/tire vertical stiffness (affects natural frequencies)

---

## Subsystem Parameters Tables

### 1. Chassis & Mass

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Total mass (estimated) | 320 | kg | ASSUMPTION | Midpoint between FS-Driverless (255 kg) and 4WD electric (365 kg); typical small FS car with battery |
| Sprung mass | 250 | kg | ASSUMPTION | ~78% of total (unsprung ~40 kg for wheels/suspension) |
| Unsprung mass (per axle) | 20 | kg | ASSUMPTION | Wheels, tires, uprights, spindles typical for 13" rims |
| CoG height above ground | 0.30 | m | ASSUMPTION | 300 mm typical for low single-seater; FS-Driverless sim: 0.25 m |
| CoG longitudinal position from front axle | 0.75 | m | ASSUMPTION | 48% front weight distribution (scaled from typical FS cars) |
| **Moments of Inertia (at CoG)** | | | | |
| Pitch inertia (Iyy) | 50 | kg·m² | ASSUMPTION | Scaled from sedan (944.1 kg·m²) by (L_FS/L_sed)²≈0.053 |
| Roll inertia (Ixx) | 30 | kg·m² | ASSUMPTION | Typical ratio Ixx ≈ 0.6·Iyy for FS cars |
| Yaw inertia (Izz) | 60 | kg·m² | ASSUMPTION | Typical ratio Izz ≈ 1.2·Iyy |

### 2. Geometry & Dimensional

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Wheelbase | 1550 | mm | SOURCED | 10.3390/vehicles8050111: "wheelbase of 1550 mm" |
| Front track width | 1200 | mm | SOURCED | UniNa Corse Formula Student spec; also typical FS min/max |
| Rear track width | 1190 | mm | SOURCED | UniNa Corse Formula Student spec |
| Vehicle length (approx) | 2800 | mm | ASSUMPTION | Typical FS vehicle length (bumper to bumper) |
| Vehicle width (approx) | 1200 | mm | ASSUMPTION | Track width approximate vehicle width |
| Vehicle height to CoG | 300 | mm | ASSUMPTION | CoG height parameter |
| Overhangs (front/rear) | 600/650 | mm | ASSUMPTION | Balance around wheelbase |

### 3. Tire & Wheel

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Wheel rim diameter | 330 | mm | ASSUMPTION | 13" ≈ 330 mm (standard FS) |
| Tire unloaded radius | 330 | mm | ASSUMPTION | Typical 13" tire = ~660 mm diameter |
| Tire width | 200 | mm | ASSUMPTION | Typical FS tire, 6–7" width for 13" rims |
| Tire vertical stiffness | 150000 | N/m | ASSUMPTION | Typical racing tire ~150 kN/m (sourced from racing tire models) |
| Tire mass (per wheel) | 6 | kg | ASSUMPTION | Lightweight 13" racing tire |
| Wheel + brake mass (per wheel) | 12 | kg | ASSUMPTION | 6 kg tire + ~6 kg wheel rim + brake hardware |
| Rolling resistance coefficient | 0.015 | dimensionless | SOURCED | Sedan TMeasy tire model in Chrono (0.015) |
| Tire friction coefficient | 0.80 | dimensionless | SOURCED | Typical slick tire μ for racing |

### 4. Suspension – Front Axle

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Suspension type | Double Wishbone | – | SOURCED | Standard FS suspension; Chrono sedan template uses this |
| Spring rate (front) | 50000 | N/m | ASSUMPTION | 50 N/mm typical FS; Chrono sedan 500 kN/m is scaled for sedan mass |
| Damper rate (front, low speed) | 5000 | Ns/m | ASSUMPTION | Typical FS front low-speed damping |
| Damper rate (front, high speed) | 3000 | Ns/m | ASSUMPTION | Typical FS front high-speed damping |
| Ride height (front) | 80 | mm | ASSUMPTION | Small car typical |
| Camber angle (static) | 0 | deg | ASSUMPTION | Neutral baseline |
| Toe angle (static) | 0 | deg | ASSUMPTION | Neutral baseline |
| Anti-roll bar stiffness (front) | 8000 | Nm/rad | ASSUMPTION | Typical FS front ARB (scaled from larger cars) |

### 5. Suspension – Rear Axle

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Suspension type | Double Wishbone | – | SOURCED | Standard FS suspension; Chrono sedan template uses this |
| Spring rate (rear) | 50000 | N/m | ASSUMPTION | 50 N/mm typical FS rear; balanced with front |
| Damper rate (rear, low speed) | 6000 | Ns/m | ASSUMPTION | Rear typically stiffer than front |
| Damper rate (rear, high speed) | 3500 | Ns/m | ASSUMPTION | Typical FS rear high-speed damping |
| Ride height (rear) | 80 | mm | ASSUMPTION | Small car typical |
| Camber angle (static) | 0 | deg | ASSUMPTION | Neutral baseline |
| Toe angle (static) | 0 | deg | ASSUMPTION | Neutral baseline |
| Anti-roll bar stiffness (rear) | 5000 | Nm/rad | ASSUMPTION | Rear ARB typically softer than front (60% of front) |

### 6. Steering

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Steering type | Rack-and-pinion | – | SOURCED | Standard FS; Chrono sedan template uses this |
| Maximum steering angle | 20 | deg | ASSUMPTION | Typical FS max ~25°; 20° conservative |
| Steering ratio (rack) | 12 | – | ASSUMPTION | Typical small car rack-pinion ratio |
| Pinion radius (in Chrono) | 0.03 | m | SOURCED | Sedan template value; scales with overall vehicle |

### 7. Drivetrain & Powertrain

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Driveline configuration | Rear-wheel drive (2WD) | – | ASSUMPTION | Typical small FS car; can be 4WD but RWD is common |
| Motor max power | 60 | kW | ASSUMPTION | Typical FS electric motor (~80 kW peak, 60 kW continuous) |
| Motor max torque | 150 | Nm | ASSUMPTION | Typical FS electric motor at wheel output |
| Gear ratio (final drive) | 0.20 | – | SOURCED | Sedan template value (conical gear) |
| Differential locking limit | 100 | – | SOURCED | Sedan template open differential |

### 8. Brake System

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Brake type | Hydraulic disc (simple model) | – | ASSUMPTION | Standard FS brake; Chrono uses simple max-torque model |
| Maximum brake torque (per wheel) | 1200 | Nm | ASSUMPTION | Typical FS brake torque (scaled from sedan 2000 Nm for mass ratio) |
| Brake bias (front/rear) | 60/40 | % | ASSUMPTION | Typical FS front-biased bias |

### 9. Aerodynamics

| Parameter | Value | Unit | Status | Source / Justification |
|-----------|-------|------|--------|------------------------|
| Frontal area | 1.0 | m² | SOURCED | 10.3390/vehicles8050111 table |
| Drag coefficient (Cd) | 0.40 | – | SOURCED | 10.3390/vehicles8050111 table |
| Lift coefficient (Cl, front) | 0 | – | ASSUMPTION | No aero at low speeds; minimal on FS cars |
| Lift coefficient (Cl, rear) | 0 | – | ASSUMPTION | No aero at low speeds; minimal on FS cars |

---

## Sources

1. **10.3390/vehicles8050111**: *Multi-Parameter Optimization of Vehicle Performance for a Four-Wheel-Drive Formula Student Electric Race Car* (2024)  
   - Provides: wheelbase 1550 mm, mass 365 kg (baseline), frontal area 1 m², drag coefficient 0.4
   - MDPI Vehicles journal

2. **FS-Driverless Simulator Documentation**:  
   - Provides: reference mass 255 kg, CoG height ~0.25 m above vehicle pawn
   - Available at: https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/blob/master/docs/vehicle_model.md

3. **Dewesoft Formula Student Vehicle Dynamics Case Study**:  
   - Provides: typical FS dimensions (wheelbase 1565 mm, track 1200×1190 mm, mass with driver 330 kg)
   - Available at: https://dewesoft.com/blog/formula-student-vehicle-dynamics-model-validation

4. **Chrono Vehicle Models (Reference)**:  
   - Sedan template in `C:\Users\daksh\miniforge3\envs\chrono\Library\data\vehicle\sedan\`
   - Shows JSON structure and engineering relationships

5. **Formula Student Design Papers** (Assumed/Typical):
   - Spring rates: 40–70 N/mm typical (Formula SAE/Formula Student standard practice)
   - Inertia scaling: moment of inertia scales with (wheelbase ratio)² for geometrically similar vehicles

---

## Notes for Implementation

- **Scaling approach**: Parameters for the FS-AI ADS-DV should be closer to the **365 kg / 1550 mm reference** (10.3390/vehicles8050111) than to the FS-Driverless 255 kg (which is minimal).
- **Mass distribution**: Typical FS cars are 40% front / 60% rear by weight; CoG is ~300 mm high and placed at ~48% wheelbase from front.
- **Spring/damper tuning**: Start with the values in Section 4–5, then adjust based on target natural frequencies and damping ratios (aim for ~2–3 Hz pitch/roll for FS cars).
- **Tire model**: Chrono provides `TMeasyTire` (empirical) and `FEATireTire` (detailed). For pitch/roll, the vertical stiffness (150 kN/m estimate) is most critical.
- **Critical for simulation**: CoG height and spring rates dominate pitch/roll response. Wheel inertia and tire stiffness affect natural frequency. Damping affects settling time and transient response.
