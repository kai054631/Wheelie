#!/usr/bin/env python3
"""
Find_QR_Weights.py
==================
Back-calculates Q/R LQR weights that reproduce the hardware-validated
K values for each servo angle profile.

Target K values (from working profiles in state_space_calculation.cpp):
    servo=25:  K1=-8.0   K2=-10.0  K3=52.07  K4=2.40
    servo=45:  K1=-8.0   K2=-12.0  K3=56.60  K4=2.40
    servo=65:  K1=-10.0  K2=-15.0  K3=59.79  K4=2.77
    servo=85:  K1=-8.0   K2=-13.0  K3=61.00  K4=4.00

Strategy:
  - Fix R_EFFORT=1 (LQR only cares about Q/R ratios).
  - Search over log-space Q values using scipy.optimize.minimize.
  - Weight the cost toward K3 and K4 (the balancing-critical gains).
  - K2 is known to be poorly modelled (velocity feedback); the hardware
    value is set manually, so we relax its weight in the fit.
"""

import numpy as np
import control as ct
from scipy.optimize import minimize, differential_evolution

# ── Physical parameters (must match State_Space_Calculation.py) ────────────
BODY_MASS_KG   = 0.504
WHEEL_MASS_KG  = 0.0345
WHEEL_RADIUS_M = 0.026
PENDULUM_PIVOT_M  = 0.3
PENDULUM_PERIOD_S = 1.15
KT  = 0.04334
RPH = 2.9

g  = 9.81
R  = WHEEL_RADIUS_M
mb = BODY_MASS_KG
mw = WHEEL_MASS_KG
_d = PENDULUM_PIVOT_M
_T = PENDULUM_PERIOD_S

Jb    = mb * _d * (g * _T**2 / (4 * np.pi**2) - _d)
Jw    = 0.5 * mw * R**2
M_eff = mb + 2*mw + 2*Jw / R**2
Cm    = 2 * KT / (RPH * R)
Cv    = 2 * KT**2 / (RPH * R**2)


def build_AB(servo_deg):
    lb    = (1.385 * servo_deg + 14.575) / 1000
    J_eff = mb * lb**2 + Jb
    D     = M_eff * J_eff - (mb * lb)**2
    C1 = (mb**2) * g * lb**2 / D
    C2 = J_eff / D
    C3 = M_eff * mb * g * lb / D
    C4 = mb * lb / D
    A = np.array([
        [0,  1,       0,         0       ],
        [0, -C2*Cv,  -C1,     C2*Cv*R    ],
        [0,  0,       0,         1       ],
        [0,  C4*Cv,   C3,    -C4*Cv*R    ],
    ])
    B = np.array([[0], [-C2*Cm], [0], [-C4*Cm]])
    return A, B


def compute_K(servo_deg, q_pos, q_vel, q_pitch, q_rate, r_eff):
    A, B = build_AB(servo_deg)
    Q = np.diag([q_pos, q_vel, q_pitch, q_rate])
    try:
        K_lqr, _, _ = ct.lqr(A, B, Q, [[r_eff]])
        return -K_lqr[0]   # firmware sign convention
    except Exception:
        return np.array([np.nan]*4)


def fit_profile(servo_deg, K_target, weights=(1, 0.2, 2, 2)):
    """
    Find Q diag values (fixing R=1) that best reproduce K_target.
    weights: (K1, K2, K3, K4) — K2 is down-weighted, K3/K4 up-weighted.
    """
    K_t = np.array(K_target)

    def cost(log_q):
        q = np.exp(log_q)          # keep all Q values positive
        K = compute_K(servo_deg, q[0], q[1], q[2], q[3], 1.0)
        if np.any(np.isnan(K)):
            return 1e9
        err = (K - K_t) * weights
        return float(np.dot(err, err))

    # Search bounds: each Q in [1e-3, 1e6]
    bounds = [(-7, 14)] * 4

    result = differential_evolution(cost, bounds, seed=42,
                                    maxiter=2000, tol=1e-10,
                                    workers=1, polish=True)
    q_best = np.exp(result.x)
    K_best = compute_K(servo_deg, *q_best, 1.0)
    return q_best, K_best, result.fun


# ── Target profiles ────────────────────────────────────────────────────────
profiles = [
    # (servo_deg, K1_target, K2_target, K3_target, K4_target)
    (25,  -8.0, -10.0, 52.07, 2.40),
    (45,  -8.0, -12.0, 56.60, 2.40),
    (65, -10.0, -15.0, 59.79, 2.77),
    (85,  -8.0, -13.0, 61.00, 4.00),
]

print()
print("=" * 70)
print("  Back-calculating Q/R weights for each servo angle profile")
print("  (K2 fit is relaxed — set K2 manually on hardware)")
print("=" * 70)

for servo_deg, K1t, K2t, K3t, K4t in profiles:
    print(f"\n  servo={servo_deg}  target K=[ {K1t}, {K2t}, {K3t}, {K4t} ]")
    q, K_fit, err = fit_profile(servo_deg, [K1t, K2t, K3t, K4t])
    print(f"  Fitted K:  K1={K_fit[0]:7.3f}  K2={K_fit[1]:7.3f}  "
          f"K3={K_fit[2]:7.3f}  K4={K_fit[3]:7.3f}  (residual={err:.4f})")
    print(f"  Q = diag([{q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}])  R = 1.0")
    print(f"  -> paste into State_Space_Calculation.py:")
    print(f"     Q_POSITION={q[0]:.1f}  Q_VELOCITY={q[1]:.1f}  "
          f"Q_PITCH={q[2]:.1f}  Q_PITCH_RATE={q[3]:.1f}  R_EFFORT=1.0")

print()
print("=" * 70)
print("  NOTE: Q/R ratios are what matter to LQR, not absolute values.")
print("  You can scale all Q by the same factor (or change R_EFFORT)")
print("  without changing K.  The values above use R=1 as reference.")
print("=" * 70)
print()
