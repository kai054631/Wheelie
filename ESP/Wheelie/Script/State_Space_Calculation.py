import numpy as np
import control as ct

# --- 1. Mechanical and Electrical Constants ---
# Derived from physical hardware mass and inertia tensors
C1 = 25.69
C2 = 5.54
C3 = 532.57
C4 = 83.15

# Motor constants (multiplied by 2 to account for both 2804 wheels)
Cm1 = 0.0049 * 2
Cm2 = 0.000216 * 2
R_wheel = 0.026  # Wheel radius in meters

# --- 2. State-Space Matrices ---
# State vector X = [Position, Velocity, Pitch Angle, Pitch Velocity]
A = np.array([
    [0, 1, 0, 0],
    [0, -(C2 * Cm2) / R_wheel, -C1, C2 * Cm2],
    [0, 0, 0, 1],
    [0, (C4 * Cm2) / R_wheel, C3, -(C4 * Cm2)]
])

B = np.array([
    [0],
    [C2 * Cm1],
    [0],
    [-C4 * Cm1]
])

# --- 3. LQR Tuning Weights ---
# Q penalizes state errors: [Position, Velocity, Pitch Angle, Pitch Velocity]
Q = np.diag([100, 1 , 4, 1])

# R penalizes control effort (Motor Voltage).
# Set to 1000 to heavily penalize voltage usage and force realistically 
# achievable gains for the direct-drive gimbal motors.
R = np.array([[10]])

# --- 4. Calculate LQR Gains ---
# The lqr function returns the Gain matrix (K), solution to Riccati equation (S), 
# and the eigenvalues of the closed loop system (E).
K, S, E = ct.lqr(A, B, Q, R)

print("--- System Matrix A ---")
print(np.round(A, 4))

print("\n--- Input Matrix B ---")
print(np.round(B, 4))

print("\n--- Optimal LQR K-Matrix Gains ---")
# The output K is a 2D array, we select the first row for a 1D list of gains
formatted_gains = [round(gain, 4) for gain in K[0]]
print(f"K1 (Position):     {formatted_gains[0]}")
print(f"K2 (Velocity):     {formatted_gains[1]}")
print(f"K3 (Pitch Angle):  {formatted_gains[2]}")
print(f"K4 (Pitch Rate):   {formatted_gains[3]}")