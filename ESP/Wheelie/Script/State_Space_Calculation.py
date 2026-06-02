#!/usr/bin/env python3
"""
Wheelie LQR Gain Calculator
============================
CoG model from measurement (R²=0.9993):
    CoG_mm = 1.385 × servo_deg + 14.575

A/B matrices are derived from first principles using the measured CoG height.
Change SERVO_DEG and the K values update automatically.

Edit only Sections 1 and 1b, then run:
    python State_Space_Calculation.py
"""

import numpy as np
import control as ct

# ╔══════════════════════════════════════════════════════════════╗
# ║       SECTION 1: SERVO & LQR WEIGHTS — edit these           ║
# ╚══════════════════════════════════════════════════════════════╝

SERVO_DEG = 25   # operating angle (°) — CoG height is calculated automatically

# LQR weights  [position, velocity, pitch, pitch_rate]
# Higher Q_i → stronger correction for that state.
# Raise R_EFFORT to scale all gains down (less aggressive).
#
# Calibrated defaults (Q_POSITION=320, R=5) → K1=-8 at every servo angle.
# K3 and K4 increase naturally as servo angle rises (higher CoG = more unstable).
# K2 from this model is small (~-1); hardware may need K2 more negative (e.g. -10 to -15)
# — tune K2 manually on the robot if it drifts in velocity.
Q_POSITION   = 320.0   # x1 — wheel position   (m)   → sets K1 magnitude
Q_VELOCITY   = 300.0   # x2 — wheel velocity   (m/s) → sets K2 (model underestimates; tune manually)
Q_PITCH      =   1.0   # x3 — pitch angle      (rad) → leave unchanged
Q_PITCH_RATE =   8.0   # x4 — pitch rate       (rad/s)
R_EFFORT     =   5.0   # control effort penalty (5 → K3≈52, K4≈2.2 at servo=25°)

# ╔══════════════════════════════════════════════════════════════╗
# ║       SECTION 1b: PHYSICAL PARAMETERS — measure once        ║
# ╚══════════════════════════════════════════════════════════════╝

BODY_MASS_KG   = 0.504    # body mass without wheels (kg)
WHEEL_MASS_KG  = 0.0345   # mass of ONE wheel (kg)
WHEEL_RADIUS_M = 0.026    # wheel radius (m)

# Moment of inertia via physical pendulum:
#   Hang body from pivot at distance d above CoM, time 20 full swings → T
PENDULUM_PIVOT_M  = 0.036   # d — pivot-to-CoM distance (m)
PENDULUM_PERIOD_S = 0.43    # T — period of one full swing (s)

# Motor drive constants (empirical, validated on hardware — 2804 gimbal, both wheels)
Cm1 = 0.02989      # net force per volt        (N/V)
Cm2 = 0.001295    # back-EMF viscous damping  (N·s/m per V)

# ╔══════════════════════════════════════════════════════════════╗
# ║       SECTION 2: COMPUTATION — do not edit below            ║
# ╚══════════════════════════════════════════════════════════════╝

g  = 9.81
R  = WHEEL_RADIUS_M
mb = BODY_MASS_KG
mw = WHEEL_MASS_KG

# CoG height from measured servo-angle data (R²=0.9993)
lb = (1.385 * SERVO_DEG + 14.575) / 1000.0   # m

# Moment of inertia about body CoM (physical pendulum formula)
_d  = PENDULUM_PIVOT_M
_T  = PENDULUM_PERIOD_S
Jb  = mb * _d * (g * _T**2 / (4 * np.pi**2) - _d)

# Effective mechanical parameters
Jw    = 0.5 * mw * R**2
M_eff = mb + 2*mw + 2*Jw / R**2
J_eff = mb * lb**2 + Jb
D     = M_eff * J_eff - (mb * lb)**2

# First-principles A/B coefficients derived from CoG height
C1 = (mb**2) * g * lb**2 / D    # A[1,2] — pitch → linear accel coupling
C2 = J_eff   / (D * R)          # A[1,1], A[1,3] — back-EMF scaling (translation)
C3 = M_eff * mb * g * lb / D    # A[3,2] — gravity (inverted pendulum instability)
C4 = mb * lb / (D * R)          # A[3,1], A[3,3] — back-EMF scaling (rotation)

# State: x = [position(m), velocity(m/s), pitch(rad), pitch_rate(rad/s)]
A = np.array([
    [0,  1,              0,        0       ],
    [0, -(C2*Cm2)/R,   -C1,     C2*Cm2    ],
    [0,  0,              0,        1       ],
    [0,  (C4*Cm2)/R,    C3,    -(C4*Cm2)  ],
])

# B[1] is negative: positive voltage drives wheels forward which registers as
# decreasing encoder position in the firmware's sign convention.
B = np.array([
    [ 0         ],
    [-C2 * Cm1  ],
    [ 0         ],
    [-C4 * Cm1  ],
])

Q     = np.diag([Q_POSITION, Q_VELOCITY, Q_PITCH, Q_PITCH_RATE])
R_lqr = np.array([[R_EFFORT]])

K, _, E = ct.lqr(A, B, Q, R_lqr)
K = -K[0]   # negate: firmware uses voltage = -(K·x), motor command = K·x

# ── Output ─────────────────────────────────────────────────────
SEP = "═" * 60
print()
print(SEP)
print(f"  Wheelie LQR  │  servo={SERVO_DEG}°   CoG={lb*1000:.1f} mm  (first-principles)")
print(SEP)
print(f"  K1  (position,   x1):   {K[0]:>9.4f}   →  main.cpp  K1")
print(f"  K2  (velocity,   x2):   {K[1]:>9.4f}   →  main.cpp  K2")
print(f"  K3  (pitch,      x3):   {K[2]:>9.4f}   →  main.cpp  K3")
print(f"  K4  (pitch rate, x4):   {K[3]:>9.4f}   →  main.cpp  K4")
print(SEP)
print(f"  Derived constants:")
print(f"    lb={lb*1000:.2f}mm   Jb={Jb:.5f}kg·m²   M_eff={M_eff:.4f}kg")
print(f"    C1={C1:.2f}   C2={C2:.2f}   C3={C3:.2f}   C4={C4:.2f}")
print(f"    C4/C2={C4/C2:.3f}   K4/K3={K[3]/K[2]:.4f}")
print(SEP)
print(f"  Closed-loop eigenvalues:")
for ev in E:
    print(f"    {ev:.4f}")
print(SEP)

warnings = []
if abs(K[3]) >= 4.0:
    warnings.append(f"  ⚠  K4 = {K[3]:.2f} — raise R_EFFORT to reduce.")
if any(ev.real >= 0 for ev in E):
    warnings.append("  ⚠  Unstable eigenvalue — check Q/R values.")
for w in warnings:
    print(w)
if warnings:
    print(SEP)

print()
print(f"  Q = diag([{Q_POSITION}, {Q_VELOCITY}, {Q_PITCH}, {Q_PITCH_RATE}])   R = {R_EFFORT}")
print()
