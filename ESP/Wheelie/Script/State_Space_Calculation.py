# include "config.h"
import numpy as np
import scipy.linalg as la
import control

# // --- State Space Variable ---
Wr = 0.026 #in meters, wheel radius
Mr =0.038 #in kg, mass of the wheel
Jr=0.00002569 #in kg*m^2, moment of inertia of the wheel
Fr=0.0 #in N*s/m, friction coefficient of the wheel
l=0.05 #in meters, distance from the wheel to the center of mass
Mb=0.63 #in kg, mass of the body
Jb=0.001157 #in kg*m^2, moment of inertia of the body
Fb=0.0 #in N*s/m, friction coefficient of the body
g=9.81 # in m/s^2, acceleration due to gravity

# // --- MOTOR PARAMETERS ---
RM=6.0 # Motor Resistance in Ohms
KM=0.0434 # Motor Torque Constant
Ke=0.0434 # Motor Torque Constant
KV=220 # Motor KV rating in rpm/V 

# =================================================================
# 2. 状态空间单侧通道模型推导 (A 与 B 矩阵)
# =================================================================
# 拉格朗日方程的特征行列式分母 p
p = Jr*(Jb + Mb*l**2) + Jb*(Mr + Mb)*Wr**2 + Mr*Mb*(l**2)*(Wr**2)

# 计算电机反电动势阻尼和力矩传导的中间转化项
alpha = (2 * KM * Ke) / (RM * Wr**2)
beta  = (2 * KM * Ke) / (RM * Wr)
gamma = (2 * KM) / RM

# 计算连续系统矩阵 A 的各项
A22 = (-alpha * (Jb + Mb*l**2) * Wr**2 - beta * Mb*l*Wr**2) / p
A23 = (Mb**2 * g * l**2 * Wr**2) / p
A42 = (alpha * Mb*l*Wr + beta * (Mr + Mb)*Wr) / p
A43 = (Mb*g*l * (Jr + (Mr + Mb)*Wr**2)) / p

A = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, A22, A23, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, A42, A43, 0.0]
])

# 计算控制输入矩阵 B 的各项
B21 = (gamma * (Jb + Mb*l**2) + gamma * Mb*l*Wr) / p
B41 = (-gamma * Mb*l - gamma * (Mr + Mb)*Wr) / p

B = np.array([
    [0.0],
    [B21],
    [0.0],
    [B41]
])

# =================================================================
# 3. LQR 代数里卡蒂方程求解
# =================================================================
# 矩阵 Q 惩罚权重配置: [小车位移, 位移速度, 车体倾摆角度, 陀螺仪角速度]
Q = np.diag([40.0, 2.5, 150.0, 18.0])  # 进一步调大倾角和定位惩罚，让锁死能力更坚硬
# 矩阵 R 电压控制开销惩罚: 针对 6 欧姆低内阻防震荡和发热
R = np.array([[0.3]])                 

# 计算 LQR 最佳增益值
K, S, E = control.lqr(A, B, Q, R)

print("=========================================================")
print("      WHEELIE 自平衡小车 LQR 矩阵点乘增益向量计算结果         ")
print("=========================================================")
print(f"const float k1 = {K[0][0]:.4f};  // 轮子水平位置反馈 (Position)")
print(f"const float k2 = {K[0][1]:.4f};  // 轮子水平速度反馈 (Velocity)")
print(f"const float k3 = {K[0][2]:.4f};  // 车身倾斜角度反馈 (Pitch Angle in Rad)")
print(f"const float k4 = {K[0][3]:.4f};  // 车身陀螺仪角速度反馈 (Gyro Rate in Rad/s)")
print("=========================================================")

# const float k1 = -11.5470;  // 轮子水平位置反馈 (Position)
# const float k2 = 225.6896;  // 轮子水平速度反馈 (Velocity)
# const float k3 = 74.6579;  // 车身倾斜角度反馈 (Pitch Angle in Rad)
# const float k4 = 8.5337;  // 车身陀螺仪角速度反馈 (Gyro Rate in Rad/s)
