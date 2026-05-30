#!/usr/bin/env python3
"""
Wheelie LQR Gain Calculator  (6-state: longitudinal + yaw)
==========================================================
Fill in Section 1, then run:
    python State_Space_Calculation_6state.py

Outputs signed gains ready to paste into main.cpp:
  K1, K2  — wheel position / velocity   (usually negative; disabled at 25 deg)
  K3, K4  — pitch angle / pitch rate      (positive in firmware)
  K5, K6  — yaw angle / yaw rate          (NEW — for the yaw controller)

Longitudinal and yaw are decoupled (JOE/Grasser structure), so they are
two independent LQR solves. K5,K6 are shared across all leg-height rows.
"""

import numpy as np
import control as ct

# ╔══════════════════════════════════════════════════════════════╗
# ║          SECTION 1: ROBOT PARAMETERS — edit these           ║
# ╚══════════════════════════════════════════════════════════════╝

SERVO_DEG = 25                 # operating point (°) — change per row
COM_HEIGHT_MM = 46

BODY_MASS_KG = 0.45 + 0.054    # body mass without wheels (kg)

# Pitch-axis pendulum -> Jb.  Keep d small (0.03–0.05 m). d=0.3 sits at the
# point where g*T^2/4pi^2 ≈ d, so Jb -> 0 and is hyper-sensitive to T.
PENDULUM_PIVOT_DIST_M = 0.3    # d (m)
PENDULUM_PERIOD_S     = 1.15   # T (s)
if PENDULUM_PIVOT_DIST_M is not None and PENDULUM_PERIOD_S is not None:
    _d, _T = PENDULUM_PIVOT_DIST_M, PENDULUM_PERIOD_S
    BODY_MOI_JB = BODY_MASS_KG * _d * (9.81 * _T**2 / (4*np.pi**2) - _d)
else:
    BODY_MOI_JB = 0.004

WHEEL_MASS_KG  = 0.0145 + 0.020   # one wheel incl. tyre/hub (kg)
WHEEL_RADIUS_M = 0.026
TRACK_WIDTH_M  = 0.183            # wheel-centre to wheel-centre (yaw)

MOTOR_KT         = 0.04334
MOTOR_RESISTANCE = 2.55

# Yaw-axis inertia -> Jbz, bifilar pendulum: Jbz = mb*g*b^2*T^2/(4pi^2 L)
# Leave as None to use the geometric box estimate.
BIFILAR_HALF_SPACING_M = None     # b (m)
BIFILAR_LENGTH_M       = None     # L (m)
BIFILAR_PERIOD_S       = None     # T (s)

# LQR weights — longitudinal
Q_POSITION, Q_VELOCITY, Q_PITCH, Q_PITCH_RATE, R_EFFORT = 10.0, 1.0, 800.0, 8.0, 10.0
# LQR weights — yaw (gentle, never fight the balancer)
Q_YAW, Q_YAW_RATE, R_YAW = 2.0, 0.2, 1.0

# ╔══════════════════════════════════════════════════════════════╗
# ║          SECTION 2: COMPUTATION — do not edit below         ║
# ╚══════════════════════════════════════════════════════════════╝
g = 9.81
lb = COM_HEIGHT_MM/1000; mb = BODY_MASS_KG; Jb = BODY_MOI_JB
mw = WHEEL_MASS_KG; R = WHEEL_RADIUS_M; d = TRACK_WIDTH_M
Kt = MOTOR_KT; Rph = MOTOR_RESISTANCE

Jw    = 0.5*mw*R**2
M_eff = mb + 2*mw + 2*Jw/R**2
Cv    = 2*Kt**2/(Rph*R**2);  Cm  = 2*Kt/(Rph*R)     # both motors
Cv1   =   Kt**2/(Rph*R**2);  Cm1 =   Kt/(Rph*R)     # per wheel

# ---- Longitudinal block ----
J_eff = mb*lb**2 + Jb
D     = M_eff*J_eff - (mb*lb)**2
A = np.array([[0,1,0,0],
              [0,-J_eff*Cv/D, mb**2*g*lb**2/D, 0],
              [0,0,0,1],
              [0,-mb*lb*Cv/D, M_eff*mb*g*lb/D, 0]])
B = np.array([[0],[-J_eff*Cm/D],[0],[-mb*lb*Cm/D]])    # negated -> firmware sign
K,_,_ = ct.lqr(A, B, np.diag([Q_POSITION,Q_VELOCITY,Q_PITCH,Q_PITCH_RATE]),
               np.array([[R_EFFORT]]))
K_fw = -K[0]

# ---- Yaw block ----
if None not in (BIFILAR_HALF_SPACING_M, BIFILAR_LENGTH_M, BIFILAR_PERIOD_S):
    Jbz = mb*g*BIFILAR_HALF_SPACING_M**2*BIFILAR_PERIOD_S**2/(4*np.pi**2*BIFILAR_LENGTH_M)
    jbz_src = "bifilar measurement"
else:
    Jbz = mb/12.0*(0.100**2 + 0.060**2); jbz_src = "geometric estimate"
J_psi = Jbz + (d**2/2.0)*(mw + Jw/R**2)
Ay = np.array([[0,1],[0,-Cv1*d**2/(2*J_psi)]])
By = np.array([[0],[-d*Cm1/J_psi]])                    # negated -> firmware sign
Ky,_,_ = ct.lqr(Ay, By, np.diag([Q_YAW,Q_YAW_RATE]), np.array([[R_YAW]]))
Ky_fw = -Ky[0]

SEP = "═"*56
print(); print(SEP)
print(f"  Wheelie 6-state LQR  │  servo = {SERVO_DEG}°   lb = {lb*1000:.1f} mm")
print(SEP)
print(f"  K1  (position,   x1):   {K_fw[0]:>9.4f}   →  main.cpp  K1")
print(f"  K2  (velocity,   x2):   {K_fw[1]:>9.4f}   →  main.cpp  K2")
print(f"  K3  (pitch,      x3):   {K_fw[2]:>9.4f}   →  main.cpp  K3")
print(f"  K4  (pitch rate, x4):   {K_fw[3]:>9.4f}   →  main.cpp  K4")
print(f"  K5  (yaw,        x5):   {Ky_fw[0]:>9.4f}   →  main.cpp  K5")
print(f"  K6  (yaw rate,   x6):   {Ky_fw[1]:>9.4f}   →  main.cpp  K6")
print(SEP)
if abs(K_fw[3]) >= 4.0:
    print(f"  ⚠  K4 = {K_fw[3]:.2f} exceeds hardware limit 4.0 — lower Q_PITCH_RATE"); print(SEP)
print()
print("  Physical constants:")
print(f"    mb    = {mb:.4f} kg      M_eff = {M_eff:.4f} kg")
print(f"    lb    = {lb*1000:.2f} mm      J_eff = {J_eff:.6f} kg·m²")
print(f"    Jb    = {Jb:.6f} kg·m²   D     = {D:.6f}")
print(f"    Jbz   = {Jbz:.6f} kg·m²  ({jbz_src})")
print(f"    J_psi = {J_psi:.6f} kg·m²")
print(f"    Cm    = {Cm:.4f} N/V   Cv    = {Cv:.4f} N·s/m")
print()
