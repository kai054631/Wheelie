#include "config.h"

// 2. 将之前计算出的 K 增益向量作为常量放入代码
const float K1 = -12.2474 / 15; // 轮子水平位置反馈 (Position)
const float K2 = 275.986 / 15;  // 轮子水平速度反馈 (Velocity)
const float K3 = 91.2049 / 1;   // 车身倾斜角度反馈 (Pitch Angle in Rad)
const float K4 = 10.4469 / 4;   // 车身陀螺仪角速度反馈 (Gyro Rate in Rad/s)

float x1 = 0; // error in position, in meters
float x2 = 0; // error in velocity, in meters per second
float x3 = 0; // error in pitch angle, in radians
float x4 = 0; // error in gyro rate, in radians per second

//--- auto calibrate angle offset for the pitch angle ---
float average_speed = 0; 

// 3. 计算平衡电压的函数
float compute_LQR_balancing_voltage(RobotState current, RobotState target, float angle_offset_deg)
{
  // 转换为弧度
  float angle_offset_rad = angle_offset_deg * (PI / 180.0f);

  // 计算误差向量
  x1 = current.position - target.position;
  x2 = current.velocity - target.velocity;
  x3 = current.pitch_angle - angle_offset_rad - target.pitch_angle;
  x4 = current.gyro_rate - target.gyro_rate;

  // 状态空间核心：u = -(K*x)
  // 注意：这里手动加上负号，实现负反馈
  float voltage = (K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);
  return constrain(voltage, -6.0f, 6.0f);
}

// 4. 辅助函数（用于获取米和米/秒）
float get_average_distance_meters()
{
  // 假设轮半径 r = 0.026m
  return (encoderL.getAngle() + encoderR.getAngle()) / 2.0f * 0.026f;
}

float get_average_velocity_mps()
{
  return (motorL.shaft_velocity + motorR.shaft_velocity) / 2.0f * 0.026f;
}
