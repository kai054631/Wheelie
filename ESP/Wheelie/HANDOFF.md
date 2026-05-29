# Wheelie — All-Terrain Wheel-Legged Hybrid Robot
### Project Handoff Document
**Last updated:** May 2026 (battery relocation + servo-45° tuning session)
**Repository:** https://github.com/kai054631/Wheelie
**Status:** Hardware assembled, balancing functional at servo 25°, servo 45° under re-tuning after battery relocation

---

## 1. Project Overview

Wheelie is a two-wheel self-balancing (TWSB) robot with a servo-driven leg mechanism that raises and lowers the robot body. The primary goals are:

- **Stable balancing** at multiple body heights (squat → stand)
- **Squat-down / stand-up** transition while remaining balanced
- **All-terrain traversal** over uneven surfaces using the leg mechanism to absorb height variation

The control approach uses **LQR state-space control** (not PID) with **gain scheduling** across operating points as body height changes.

---

## 2. Hardware Specification

### 2.1 Electronics

| Component | Part | Notes |
|---|---|---|
| MCU | ESP32-S3 | Dual-core, FreeRTOS. Core 0 = balance + servo + monitor. Core 1 = motor FOC |
| IMU | MPU-6050 | I2C, SDA=GPIO2, SCL=GPIO1, 400 kHz. Kalman-filtered pitch angle |
| Drive motor driver | SimpleFOC Mini v1.0 | Two units, one per motor. Torque mode only |
| Wheel encoders | MT6701 × 2 | ABZ mode, **1024 PPR** (4096 counts/rev in quadrature). No Z-pin wired |
| Servo | MG996R × 2 | Left and right leg servos, mirrored (min/max swapped) |
| Power regulator | MP2236 | 6 A continuous, 3S LiPo → 5 V for ESP32-S3 + both servos |
| Battery | 3S LiPo | 11.1 V nominal, 12.6 V fully charged |

### 2.2 Drive Motors

| Parameter | Value |
|---|---|
| Model | 2804 gimbal BLDC |
| KV | 220 RPM/V |
| Phase resistance | 2.55 Ω |
| Pole pairs | 7 (14 poles) |
| Torque constant Kt | 0.04334 Nm/A |
| Back-EMF constant Kb | 0.04334 V·s/rad (= Kt in SI) |
| Peak torque @ 5 A | 0.217 Nm per motor, 0.434 Nm total |
| Max no-load speed | 2772 RPM @ 12.6 V |

### 2.3 Mechanical

| Parameter | Value | Notes |
|---|---|---|
| Body mass mb | 630 g (no wheels) | |
| Wheel mass mw | 14.5 g each | |
| Wheel radius R | 26 mm | |
| Track width d | 183 mm (wheel centre to wheel centre) | |
| CoM height lb | **See note below** | Remeasure required — battery relocated |
| Servo angle range | 20° (squat) → 100° (stand), constrained in firmware | |
| lb mapping (original) | `lb (mm) = 50 + (servo_deg − 20) × (90/80)` | Valid for original battery position only |

> **⚠️ Battery relocated (2026-05):** A 25 mm spacer block was added between the body and battery, moving the battery **30 mm lower** than the original design. This lowers the CoM at every servo angle by:
>
> `Δlb = (m_battery_kg / 0.630) × 30 mm`
>
> The battery mass is not yet confirmed. Until measured, use:
> `lb_new (mm) = lb_original − Δlb`
> or directly re-measure the CoM position by balancing the body on a rod.
>
> Once confirmed, update `lb_mm` in `State_Space_Calculation.py` and re-run to get new K gains.

### 2.4 Pin Map

| Signal | GPIO |
|---|---|
| I2C SDA (MPU6050) | 2 |
| I2C SCL (MPU6050) | 1 |
| Left servo | 4 |
| Right servo | 5 |
| Driver L — phA, phB, phC, EN | 14, 13, 12, 11 |
| Driver R — phA, phB, phC, EN | 16, 15, 7, 6 |
| Encoder L — A, B | 3, 46 |
| Encoder R — A, B | 17, 18 |

---

## 3. Derived Physical Parameters

These are computed from the hardware spec — do not re-derive unless hardware changes.

| Symbol | Value | Formula / Notes |
|---|---|---|
| Jw | 4.91 × 10⁻⁶ kg·m² | ½ · mw · R² (solid disk) |
| M_eff | 0.6735 kg | mb + 2·mw + 2·Jw/R² |
| Cm | 1.308 N/V | 2·Kt / (Rph · R) — net force per control volt |
| Cv | 2.179 N·s/m | 2·Kt² / (Rph · R²) — back-EMF viscous damping |
| **Jb** | **~0.004 kg·m² (estimated)** | Back-calculated from C3 at lb=78 mm. Expected range 0.0015–0.0035. **Measure via physical pendulum (§8) and replace.** |

> **Jb is the primary missing parameter.** The value 0.004 kg·m² was back-calculated from the working C3 constant and is used as a placeholder. Measure it physically to get accurate gain table values.

---

## 4. State-Space Model

### 4.1 State Vector

```
x = [ position (m),  velocity (m/s),  pitch angle (rad),  gyro rate (rad/s) ]
    [     x1,              x2,               x3,                  x4         ]
```

Control input `u` = voltage command (V), applied equally to both motors via SimpleFOC torque mode.

### 4.2 Linearised Equations of Motion (θ ≈ 0)

```
M_eff · p̈  − mb·lb · θ̈  =  Cm·u − Cv·ṗ      (wheel translation)
J_eff · θ̈  − mb·lb · p̈  =  mb·g·lb · θ       (body pitch)

where J_eff = mb·lb² + Jb
      D     = M_eff·J_eff − (mb·lb)²
```

### 4.3 A and B Matrices

```
A = [ 0,          1,                   0,  0 ]
    [ 0,  −J_eff·Cv/D,   mb²·g·lb²/D,  0 ]
    [ 0,          0,                   0,  1 ]
    [ 0,  −mb·lb·Cv/D,  M_eff·mb·g·lb/D, 0 ]

B = [ 0,  J_eff·Cm/D,  0,  mb·lb·Cm/D ]ᵀ
```

Only `Jb` is missing to make these fully numerical.

### 4.4 LQR Control Law

```
u = −K · x = −(K1·x1 + K2·x2 + K3·x3 + K4·x4)
```

LQR weight matrices (hardware-validated):

```python
Q     = np.diag([1.0, 1.0, 200.0, 3.0])   # q4=3 enforces K4 < 4 (see §6.2)
R_mat = np.array([[0.5]])
```

---

## 5. Firmware Architecture

### 5.1 FreeRTOS Task Layout

| Task | Core | Priority | Rate | Function |
|---|---|---|---|---|
| TaskMotorCode | 1 | 3 | 1 kHz | SimpleFOC `loopFOC()` + `move()`, fall detection |
| TaskBalanceCode | 0 | 2 | 500 Hz | MPU read → Kalman → LQR → `shared_motor_voltage` |
| TaskMonitorCode | 0 | 1 | 10 Hz | WebSocket telemetry, Serial print |
| TaskServoCode | 0 | 1 | 10 Hz | Servo write + gain scheduler trigger |

### 5.2 File Structure

```
ESP/src/
├── main.cpp                   — hardware init, FreeRTOS tasks
├── config.h                   — all extern declarations, struct definitions, pin defines
├── kalman.cpp                 — 1D Kalman filter for pitch angle
├── state_space_calculation.cpp — LQR computation, gain schedule table, drift integrator
└── webhandle.cpp              — WiFi AP, WebSocket, HTTP endpoints

Script/
└── State_Space_Calculation.py — LQR gain calculator (parameterised by servo angle / lb)
```

### 5.3 Key Data Flow

```
MPU6050 (I2C 400kHz)
    ──► kalmanUpdate()      → Pitch_angle (deg) → x3 (rad)
    ──► LPF α=0.08          → Pitch_gyro  (deg/s) → x4 (rad/s)

MT6701 ABZ (ISR)
    ──► encoderL/R.getAngle()      → x1 (m) via ×0.026
    ──► motorL/R.shaftVelocity()   → x2 (m/s) via ×0.026

x = [x1, x2, x3, x4]
    ──► compute_LQR_balancing_voltage() → shared_motor_voltage (V)
            ──► motorL/R.move(−v) [Core 1, 1kHz]
```

### 5.4 Web Interface

- **Connect:** WiFi SSID `Wheelie_Robot`, password `12345678`
- **Open:** `http://192.168.4.1`
- **Endpoints:**
  - `/set?type=K1&val=X` — update K1, K2, K3, K4, offset (O), servo (S) live
  - `/reset` — hard-snap `position_offset` to current position
  - `/togglegs` — enable/disable gain scheduling

### 5.5 IMU Calibration Values

The MPU-6050 calibration is hardcoded after running `calcOffsets(true, true)` at startup. Re-run calibration (temporarily uncomment `calcOffsets` in `setup()`) whenever the board or battery position changes, then copy the printed offsets back.

**Current values (post battery relocation, 2026-05):**

```cpp
mpu.setGyroOffsets(-1.484977, -0.755755, -2.337404);
mpu.setAccOffsets( -0.003804, -0.074246,  0.216630);
```

**Calibration procedure:**
1. Uncomment `mpu.calcOffsets(true, true)` in `setup()` and comment out `setGyroOffsets`/`setAccOffsets`
2. Hold robot perfectly upright and still during the ~3 s startup measurement window
3. Copy the printed values into `setGyroOffsets` / `setAccOffsets` and comment `calcOffsets` back out

---

## 6. Known Hardware Constraints & Tuning Rules

### 6.1 Gyro Rate Noise (Critical)

The 2804 gimbal motor produces mechanical vibration (cogging + FOC PWM switching) at 5–50 Hz that couples through the chassis into the MPU6050. The gyro rate (x4) is particularly sensitive.

**Fix applied:** Gyro LPF alpha reduced from 0.25 (fc = 22.9 Hz) to **0.08 (fc = 6.6 Hz)** in `TaskBalanceCode`. This suppresses motor noise before K4 amplifies it into voltage commands.

### 6.2 K4 Hardware Limit

**K4 must stay below 4.0.** Above this threshold the motor voltage commands oscillate fast enough to drive repeated acceleration/braking cycles. This is a hard empirical limit of this robot's chassis + motor combination.

**LQR is configured to output K4 ≈ 3.3** via q4 = 3.0 in the Q matrix.

> Do not raise q4 in `State_Space_Calculation.py` without first tightening the gyro LPF further (lower alpha).

### 6.3 K4 Minimum — Diverging Oscillation Warning

**K4 must be at least ~1.5.** Oscillation analysis at servo 45° with K4=0.4 showed:

- Peak pitch error amplitude grew **72% per half-cycle** (diverging, not decaying)
- Voltage saturated at ±2 V within 1.4 s, after which the robot had no corrective authority
- Full oscillation period ~1.6 s, consistent with nearly zero angular-rate damping

Root cause: K4=0.4 contributes only ~17% of total correction vs K3's 83%. The gyro rate term is critical for damping oscillations after each correction.

**Safe starting range: K4 = 1.5 – 2.5.** If robot oscillates after a push, increase K4. If robot brakes/stutters, decrease K4 or check gyro noise (§6.1).

### 6.4 Position Control (K1, K2)

K1 and K2 are disabled (= 0) at servo 25° until pitch balancing is fully stable. Position control is enabled from servo 45° onward. When re-enabling K1/K2 at any servo angle, start from 0 and increase slowly while observing drift behaviour.

### 6.5 Fall Detection

Motors auto-disable when `|pitch_angle| > 0.4 rad (≈ 23°)`. Re-enable is automatic on recovery. This threshold is safe for all lb operating points.

### 6.6 Voltage Limit

`motor.voltage_limit = 8.0 V` (below 3S max of 12.6 V). LQR output is also clamped to ±8 V in `compute_LQR_balancing_voltage`. Do not raise above 8 V without checking motor temperature under sustained load.

---

## 7. Gain Schedule Table

### 7.1 Current State

| Row | Servo | lb (original) | lb (with battery −30 mm) | K1 | K2 | K3 | K4 | angle_offset | Status |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 25° | 56 mm | ~50 mm (est.) | 0.0 | 0.0 | 54.36 | 2.30 | TBD | ✅ Manually tuned, working |
| 1 | 45° | 78 mm | ~68 mm (est.) | 0 | 0 | **55–65** | **2.0** | **−0.19** | 🔧 Re-tuning after battery relocation. Start here. |
| 2 | 73° | 109 mm | ~99 mm (est.) | TBD | TBD | TBD | TBD | TBD | ⏳ Awaiting Jb measurement |
| 3 | 100° | 140 mm | ~130 mm (est.) | TBD | TBD | TBD | TBD | TBD | ⏳ Awaiting Jb measurement |

> lb estimates for modified configuration assume ~12 mm CoM shift (corresponds to ~250 g battery in a 630 g body). Confirm by measuring battery mass or re-measuring CoM directly.

**Row 1 tuning history (servo 45°, post battery relocation):**
- K4=7.0 — above the K4<4 hardware limit, do not use
- K4=0.4 — caused diverging oscillation (see §6.3), do not use
- **K3=55–65, K4=2.0 — recommended starting point.** Tune K3 first (±5 steps) until balance holds several seconds, then tune K4 (±0.2 steps) to damp oscillation after a push.
- `angle_offset = −0.19` rad is the current firmware value; adjust via web UI `/set?type=O&val=X`

`gain_schedule_enabled = false` — gains are applied manually via web UI until all rows are validated.

### 7.2 How to Complete the Table

1. **Measure battery mass** (scale) and compute `Δlb = (m_bat / 0.630) × 30 mm`
2. **Update `lb_mm` in `State_Space_Calculation.py`** for each servo angle operating point
3. **Measure Jb** using the physical pendulum method (see §8) and replace estimated value
4. **Run:** `python State_Space_Calculation.py` for each servo angle
5. **Copy** the printed K gains into `state_space_calculation.cpp`
6. **Test each row** individually by setting servo angle via web UI, then tune `angle_offset` until the robot holds still
7. **Record** the validated angle_offset for each row in the table above
8. **Set** `gain_schedule_enabled = true` and test slow squat → stand transitions

---

## 8. Pending: Jb Measurement (Physical Pendulum)

### 8.1 What It Is

Jb = moment of inertia of the robot body about the CoM axis (pitching axis, parallel to wheel axle). This is the last unknown parameter before the full LQR model can be computed.

### 8.2 Method

1. Remove both wheels. Keep battery, ESP32-S3, SimpleFOC boards, servos and all wiring installed — measure with full electronics load including the 30 mm battery spacer
2. Balance body on a horizontal rod aligned with the left-right axis to find CoM. Mark it
3. Mount a horizontal pivot rod above the CoM (e.g. through a chassis bolt hole). Measure `d` = distance from pivot to CoM mark (aim for d ≥ 30 mm)
4. Hang body from the pivot so it swings freely in the forward-backward plane
5. Displace < 5° and time **20 complete oscillations** × 3 runs, average → period T
6. Apply formula:

```
Jb = mb × d × (g × T² / 4π² − d)
```

### 8.3 Expected Range

Based on robot geometry (630 g body, roughly 100 × 140 × 60 mm), Jb is expected between **0.0015 and 0.0035 kg·m²**. The back-calculated value of ~0.004 kg·m² is slightly above this range, which is why direct measurement is important.

---

## 9. Hardware Modifications Log

| Date | Change | Impact |
|---|---|---|
| 2026-05 | Added 25 mm spacer block between body and battery — battery is now **30 mm lower** than original design | Lowers CoM at every servo angle. Recalibrate IMU, re-tune angle_offset, recompute lb for gain table. |
| 2026-05 | IMU recalibrated with new hardcoded offsets (see §5.5) | Old offsets invalidated by body geometry change |

---

## 10. Future Work

### 10.1 Immediate (priority order)
- [ ] Measure battery mass → compute exact Δlb → update `State_Space_Calculation.py`
- [ ] Validate servo 45° balance with K3=55–65, K4=2.0, tune angle_offset
- [ ] Measure Jb via physical pendulum (§8) and replace estimated 0.004 kg·m² value
- [ ] Run `State_Space_Calculation.py` for all four servo angles and fill gain table
- [ ] Replace servo 45° gains with LQR-computed values (current empirical K4=7 exceeds hardware limit)
- [ ] Enable gain scheduling and test squat → stand transition

### 10.2 Yaw / Turning Control
Track width `d = 183 mm` is confirmed. Turning control uses differential voltage:

```cpp
// In TaskBalanceCode, after computing v:
float dv = Kyaw * (target_yaw_rate - actual_yaw_rate);
motorL.move(-(v + dv));
motorR.move(-(v - dv));

// Actual yaw rate from encoders:
float yaw_rate = (motorR.shaftVelocity() - motorL.shaftVelocity()) * R / d; // rad/s
```

Suggested starting Kyaw ≈ 0.5, tune via web UI.

### 10.3 All-Terrain / Obstacle Traversal
- Servo angle profile during step climbing (squat before obstacle, extend to step over)
- Disturbance rejection: verify LQR response to 10–20 N impulse at body
- Ground clearance mapping vs servo angle

### 10.4 IMU Placement
If chassis vibration noise remains problematic after LPF tuning, consider adding vibration-isolating foam tape between the MPU6050 PCB and chassis mounting point.

---

## 11. Reference Documents

| Document | Location | Purpose |
|---|---|---|
| FK Engineering blog | https://fkeng.blogspot.com/2019/ | TWSB theory, state-space derivation, bi-filar pendulum method |
| LACCEI 2020 paper | Project file: FP556.pdf | TWSB modeling with ESP32 + MPU6050, PID and LQR comparison |
| ETH Zürich MPC lab | Project file: SigiStudent.pdf | Predictive control of TWSB, advanced reference |
| SimpleFOC docs | https://docs.simplefoc.com | Motor + encoder setup, torque mode configuration |
| MT6701 datasheet | — | ABZ mode configuration, PPR setting via EEPROM |

---

## 12. Quick Reference — Tuning Cheat Sheet

```
SYMPTOM                          LIKELY CAUSE              FIX
────────────────────────────────────────────────────────────────────────
Wheels speed up and brake        K4 too high               Keep K4 < 4.0
Robot falls forward/backward     angle_offset wrong        Adjust via web UI /set?type=O
Robot oscillates (slow, growing) K4 too low (<1.5)         Raise K4 in steps of 0.2
Robot oscillates (slow, stable)  K3 too high               Reduce K3 by 10% steps
Robot drifts position over time  K1=0 or drift integrator  Enable K1 or press /reset
Pitch jerky/noisy                gyro LPF too wide         Reduce LPF alpha below 0.08
Motors stutter on startup        initFOC failed            Check encoder wiring + PPR = 1024
Robot stable but gains jump      gain_schedule interpolation Check servo angle range 20–100°
```
