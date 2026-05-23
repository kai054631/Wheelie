import numpy as np
import scipy.linalg  # Using standard scientific solver for Riccati Equations

# --- 1. Mechanical and Electrical Constants (UPDATED FOR 4015 MOTORS) ---
# Derived from physical hardware mass and updated inertia tensors
# accounts for updated robot mass (710g base + heavier winding hubs)
C1 = 147.1005
C2 = 5.54
C3 = 220.6507
C4 = 83.15

# Updated 4015 Motor constants 
# KV = 100 -> Ke = 1 / (100 * 2 * pi / 60) ≈ 0.0955 V/(rad/s)
# Coil Resistance Rm = 4.8 Ohms
# Multiplied inside state equations to map dual-wheel tracking parameters
Cm1 = 0.03978
Cm2 = 0.00379
R_wheel = 0.026  # Wheel radius in meters (26mm)

# --- 2. State-Space Matrices ---
# State vector X = [Position, Velocity, Pitch Angle, Pitch Velocity]
A = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, -(C2 * Cm2) / R_wheel, -C1, C2 * Cm2],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, (C4 * Cm2) / R_wheel, C3, -(C4 * Cm2)]
])

B = np.array([
    [0.0],
    [C2 * Cm1],
    [0.0],
    [-C4 * Cm1]
])

# --- 3. LQR Tuning Weights (BALANCED FOR SMOOTH FLIGHT CONTROL) ---
# Q penalizes state errors: [Position, Velocity, Pitch Angle, Pitch Velocity]
Q = np.diag([5.0, 0.5, 45.0, 2.5])

# R penalizes control effort (Motor Voltage Command).
# Increasing this suppresses aggressive over-corrections and prevents jittering.
R = np.array([[1.0]])

# --- 4. Calculate LQR Gains ---
# Solves the continuous Algebraic Riccati Equation (CARE)
P = scipy.linalg.solve_continuous_are(A, B, Q, R)
K = np.dot(scipy.linalg.inv(R), np.dot(B.T, P))

print("--- System Matrix A ---")
print(np.round(A, 4))

print("\n--- Input Matrix B ---")
print(np.round(B, 4))

print("\n--- Optimal LQR K-Matrix Gains ---")
formatted_gains = [round(gain, 4) for gain in K[0]]
print(f"const float K1 = {formatted_gains[0]:.4f};  // Position Feedback")
print(f"const float K2 = {formatted_gains[1]:.4f};  // Velocity Damping")
print(f"const float K3 = {formatted_gains[2]:.4f};  // Pitch Angle Correction")
print(f"const float K4 = {formatted_gains[3]:.4f};  // Gyro Rate Damping")