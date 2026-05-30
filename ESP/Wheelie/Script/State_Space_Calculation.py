#!/usr/bin/env python3
"""
Wheelie LQR Gain Calculator
============================
Fill in Section 1 (Robot Parameters), then run:
    python State_Space_Calculation.py

The output K values are signed and ready to paste directly into main.cpp.
  K1, K2  — wheel position / velocity  (usually negative in firmware)
  K3, K4  — pitch angle / pitch rate   (positive in firmware)
"""

import numpy as np
import control as ct

# ╔══════════════════════════════════════════════════════════════╗
# ║          SECTION 1: ROBOT PARAMETERS — edit these           ║
# ╚══════════════════════════════════════════════════════════════╝

# ── 1a. Servo angle ─────────────────────────────────────────────
SERVO_DEG = 25            # operating point (°) — change per gain-table row

COM_HEIGHT_MM = 46

# ── 1c. Body ────────────────────────────────────────────────────
BODY_MASS_KG = 0.45 + 0.054   # body mass without wheels (kg)
                               # split as body + electronics if easier to measure
PENDULUM_PIVOT_DIST_M = 0.3   # d — distance from pivot to CoM (m),  e.g. 0.035
PENDULUM_PERIOD_S     = 1.15   # T — period of one full swing   (s),  e.g. 0.48

if PENDULUM_PIVOT_DIST_M is not None and PENDULUM_PERIOD_S is not None:
    _d = PENDULUM_PIVOT_DIST_M
    _T = PENDULUM_PERIOD_S
    BODY_MOI_JB = BODY_MASS_KG * _d * (9.81 * _T**2 / (4 * np.pi**2) - _d)
else:
    BODY_MOI_JB = 0.004        # fallback estimate — replace by measuring d and T above

# ── 1d. Wheels ──────────────────────────────────────────────────
WHEEL_MASS_KG  = 0.0145+0.020       # mass of ONE wheel (kg)
WHEEL_RADIUS_M = 0.026        # wheel radius (m)

# ── 1e. Motors  (2804 gimbal — confirmed hardware values) ────────
MOTOR_KT          = 0.04334   # torque constant Kt  (Nm/A)
MOTOR_RESISTANCE  = 2.9      # phase resistance Rph (Ω)

# ── 1f. LQR weights ─────────────────────────────────────────────
# Higher Q → stronger correction for that state.
# Set Q_POSITION / Q_VELOCITY = 0 to disable wheel drift correction.
#
# ⚠  Keep Q_PITCH_RATE ≤ 3.  Above this, K4 will exceed the 4.0 hardware
#    limit (motor oscillation — HANDOFF §6.2).
Q_POSITION   =  10.0    # x1 — wheel position  (m)
Q_VELOCITY   =  1.0    # x2 — wheel velocity  (m/s)
Q_PITCH      =  800   # x3 — pitch angle     (rad)
Q_PITCH_RATE =   8   # x4 — pitch rate      (rad/s)  ← do not exceed 3

R_EFFORT     =   100.0   # control effort penalty (lower → more aggressive)


# ╔══════════════════════════════════════════════════════════════╗
# ║          SECTION 2: COMPUTATION — do not edit below         ║
# ╚══════════════════════════════════════════════════════════════╝

g   = 9.81
lb  = COM_HEIGHT_MM / 1000
mb  = BODY_MASS_KG
Jb  = BODY_MOI_JB
mw  = WHEEL_MASS_KG
R   = WHEEL_RADIUS_M
Kt  = MOTOR_KT
Rph = MOTOR_RESISTANCE

# Derived mechanical constants# Derived mechanical constants (these were already correct — now actually used)
Jw    = 0.5 * mw * R**2
M_eff = mb + 2*mw + 2*Jw/R**2     # effective total mass (kg)
Cv    = 2 * Kt**2 / (Rph * R**2)  # back-EMF viscous damping (N·s/m)
Cm    = 2 * Kt   / (Rph * R)      # net force per volt (N/V)

J_eff = mb * lb**2 + Jb
D     = M_eff * J_eff - (mb * lb)**2   # > 0 required

# State: x = [position(m), velocity(m/s), pitch(rad), pitch_rate(rad/s)]
# Documented EOM (HANDOFF §4.3):
A = np.array([
    [0,  1,                 0,                 0],
    [0, -J_eff*Cv/D,        mb**2*g*lb**2/D,   0],
    [0,  0,                 0,                 1],
    [0, -mb*lb*Cv/D,        M_eff*mb*g*lb/D,   0],
])
# B sign negated to match firmware (compute returns -(K·x), TaskMotor applies -shared)
B = np.array([[0], [-J_eff*Cm/D], [0], [-mb*lb*Cm/D]])

Q     = np.diag([Q_POSITION, Q_VELOCITY, Q_PITCH, Q_PITCH_RATE])
R_lqr = np.array([[R_EFFORT]])

K, _, _ = ct.lqr(A, B, Q, R_lqr)
K = K[0]

# Firmware sign: main.cpp computes  voltage = -(K1·x1 + K2·x2 + K3·x3 + K4·x4)
# so the firmware value for each gain is  −K_lqr.
K_fw = -K

# ── Output ──────────────────────────────────────────────────────
SEP = "═" * 56
print()
print(SEP)
print(f"  Wheelie LQR  │  servo = {SERVO_DEG}°   lb = {lb*1000:.1f} mm")
print(SEP)
print(f"  K1  (position,   x1):   {K_fw[0]:>9.4f}   →  main.cpp  K1")
print(f"  K2  (velocity,   x2):   {K_fw[1]:>9.4f}   →  main.cpp  K2")
print(f"  K3  (pitch,      x3):   {K_fw[2]:>9.4f}   →  main.cpp  K3")
print(f"  K4  (pitch rate, x4):   {K_fw[3]:>9.4f}   →  main.cpp  K4")
print(SEP)

warnings = []
if abs(K_fw[3]) >= 4.0:
    warnings.append(f"  ⚠  K4 = {K_fw[3]:.2f} exceeds hardware limit of 4.0!")
    warnings.append("     Lower Q_PITCH_RATE or raise R_EFFORT.")
for w in warnings:
    print(w)
if warnings:
    print(SEP)

print()
print("  Physical constants:")
print(f"    mb    = {mb:.4f} kg      M_eff = {M_eff:.4f} kg")
print(f"    lb    = {lb*1000:.2f} mm      J_eff = {J_eff:.6f} kg·m²")
print(f"    Jb    = {Jb} kg·m²   D     = {D:.6f}")
print(f"    Cm    = {Cm:.4f} N/V   Cv    = {Cv:.4f} N·s/m")
print()
