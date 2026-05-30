#!/usr/bin/env python3
"""
lqr_compute_6state.py  --  Wheelie 6-state LQR gain generator
=============================================================

State vector (6):
    x = [ p, p_dot, theta, theta_dot, psi, psi_dot ]
          x1   x2     x3      x4        x5    x6
    p      : forward position        (m)
    theta  : pitch angle             (rad)   -- balancing
    psi    : yaw angle               (rad)   -- heading / turning

Two control channels (decoupled, JOE-style):
    v   : COMMON-mode per-motor voltage  -> drives [p, theta]      (balancing)
    dv  : DIFFERENTIAL per-motor voltage -> drives [psi]           (yaw)
    u_R = v + dv ,  u_L = v - dv      (dv>0 => turn CCW / left)

Control law applied in firmware:
    v  = -(K1*p  + K2*p_dot + K3*theta + K4*theta_dot)
    dv = -(K5*(psi - psi_ref) + K6*psi_dot)

Because balancing and yaw are decoupled to first order for a symmetric
chassis, the 6x6 A is block-diagonal and we solve two small LQRs:
   - the existing 4-state longitudinal block (one row per leg height lb)
   - one 2-state yaw block (shared across all leg heights)

Usage:
    python lqr_compute_6state.py <Jb_kgm2> [Jbz_kgm2]
        Jb   = body MOI about the PITCH axis (the existing unknown)
        Jbz  = body MOI about the YAW (vertical) axis (the NEW unknown)
               optional; defaults to the geometric estimate below.

Requires: numpy, scipy
"""

import sys
import numpy as np
from scipy.linalg import solve_continuous_are

# ----------------------------------------------------------------------
# Hardware constants (SI) -- keep in sync with config.h / HANDOFF.md
# ----------------------------------------------------------------------
mb   = 0.504        # body mass (kg)                 [updated May 2026]
mw   = 0.0145       # wheel mass each (kg)
R    = 0.026        # wheel radius (m)
d    = 0.183        # track width, wheel-centre to wheel-centre (m)
g    = 9.81
Kt   = 0.04334      # torque constant (Nm/A) = Kb (V.s/rad)
Rph  = 2.55         # phase resistance (ohm)

Jw   = 0.5 * mw * R**2                 # wheel MOI (solid disk)
M_eff = mb + 2*mw + 2*Jw/R**2          # effective translational mass
Cm   = 2*Kt/(Rph*R)                    # net force per control volt (both motors)
Cv   = 2*Kt**2/(Rph*R**2)             # back-EMF viscous damping (both motors)
Cm1  = Kt/(Rph*R)                      # per-wheel force constant
Cv1  = Kt**2/(Rph*R**2)               # per-wheel damping

# ----------------------------------------------------------------------
# LQR weights
# ----------------------------------------------------------------------
# Longitudinal (identical to the working 4-state design: q3=200, q4=3)
Q_long = np.diag([1.0, 1.0, 200.0, 3.0])
R_long = np.array([[0.5]])

# Yaw block. Start gentle so it never fights the balancer or saturates.
Q_yaw = np.diag([2.0, 0.2])           # [psi, psi_dot]
R_yaw = np.array([[1.0]])

# Leg-height schedule rows (servo angle -> lb via lb=50+(deg-20)*90/80, mm)
ROWS = [(25, 0.056), (45, 0.078), (73, 0.109), (100, 0.140)]


def lqr(A, B, Q, Rm):
    P = solve_continuous_are(A, B, Q, Rm)
    K = np.linalg.inv(Rm) @ B.T @ P
    return K


def long_AB(lb, Jb):
    J_eff = mb*lb**2 + Jb
    D = M_eff*J_eff - (mb*lb)**2
    A = np.array([
        [0, 1,                       0,                 0],
        [0, -J_eff*Cv/D,             mb**2*g*lb**2/D,   0],
        [0, 0,                       0,                 1],
        [0, -mb*lb*Cv/D,             M_eff*mb*g*lb/D,   0],
    ])
    B = np.array([[0],
                  [J_eff*Cm/D],
                  [0],
                  [mb*lb*Cm/D]])
    return A, B


def yaw_AB(Jbz):
    # Effective yaw inertia: body about vertical axis + wheels at +/- d/2
    J_psi = Jbz + (d**2/2.0)*(mw + Jw/R**2)
    A = np.array([[0, 1],
                  [0, -Cv1*d**2/(2*J_psi)]])
    B = np.array([[0],
                  [d*Cm1/J_psi]])          # dv>0 => +psi (CCW)
    return A, B, J_psi


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    Jb = float(sys.argv[1])

    # Geometric default for Jbz: box approx, vertical axis ~ m/12*(w^2+dep^2)
    # body ~ 100mm wide (left-right) x 60mm deep (front-back)
    Jbz_default = mb/12.0*(0.100**2 + 0.060**2)
    Jbz = float(sys.argv[2]) if len(sys.argv) > 2 else Jbz_default

    A_y, B_y, J_psi = yaw_AB(Jbz)
    Ky = lqr(A_y, B_y, Q_yaw, R_yaw).flatten()
    K5, K6 = Ky[0], Ky[1]

    print("="*70)
    print(f"  Derived constants:  M_eff={M_eff:.4f} kg  Cm={Cm:.3f}  Cv={Cv:.3f}")
    print(f"  Jb  (pitch MOI) = {Jb:.6f} kg.m^2   [measure: physical pendulum]")
    print(f"  Jbz (yaw  MOI)  = {Jbz:.6f} kg.m^2  [measure: bifilar pendulum]"
          + ("  (geometric estimate)" if len(sys.argv) <= 2 else ""))
    print(f"  J_psi effective = {J_psi:.6f} kg.m^2")
    print(f"  YAW gains (shared all rows):  K5={K5:.3f}  K6={K6:.3f}")
    print("="*70)
    print()
    print("// ---- paste into state_space_calculation.cpp ----")
    print("// GainRow: {servo_deg, K1, K2, K3, K4, K5, K6}")
    print(f"const int   N_GAIN_ROWS = {len(ROWS)};")
    print("const float GAIN_TABLE[N_GAIN_ROWS][7] = {")
    for deg, lb in ROWS:
        A, B = long_AB(lb, Jb)
        K = lqr(A, B, Q_long, R_long).flatten()
        K1, K2, K3, K4 = K
        flag = "   // <-- K4 OVER 4.0 LIMIT!" if abs(K4) > 4.0 else ""
        print(f"  {{ {deg:5.1f}f, {K1:8.3f}f, {K2:8.3f}f, "
              f"{K3:8.3f}f, {K4:6.3f}f, {K5:6.3f}f, {K6:6.3f}f }},{flag}")
    print("};")
    print()
    print("Notes:")
    print(" - K1=K2 are the LQR position gains; keep them 0 until balancing is")
    print("   stable at a given row, then enable per HANDOFF 6.3.")
    print(" - K5,K6 are identical across rows (yaw inertia ~ independent of leg")
    print("   height). Tune Q_yaw if turns are too slow / too aggressive.")
    print(" - If any K4 > 4.0, tighten gyro LPF first, then lower q4 (HANDOFF 6.2).")


if __name__ == "__main__":
    main()
