#include "config.h"

//--- auto calibrate angle offset for the pitch angle ---
float average_speed = 0; 
float x1 = 0.0f;
float x2 = 0.0f;
float x3 = 0.0f;
float x4 = 0.0f;
float position_offset = 0.0f;
// 3. 计算平衡电压的函数
float compute_LQR_balancing_voltage(RobotState current, RobotState target, float angle_offset_deg)
{
  float angle_offset_rad = angle_offset_deg;

  // Slow drift: position_offset follows actual position with tau ~20s.
  // Prevents x1 from growing unbounded after a disturbance, which would
  // otherwise create a large correction voltage that fights against balance.

  // 计算误差向量
  x1 = current.position - position_offset - target.position;
  x2 = current.velocity - target.velocity;
  x3 = current.pitch_angle - angle_offset_rad - target.pitch_angle;
  x4 = current.gyro_rate - target.gyro_rate;
  // printf("angle_offset_rad: %.4f\n", angle_offset_rad);
  // Cap position error to ±0.2m so large drift doesn't dominate over pitch control
  x3 = constrain(x3, -0.2f, 0.2f);

  // 状态空间核心：u = -(K*x)
  float voltage = -((K1 * x1) + (K2 * x2) + (K3 * x3) + (K4 * x4));
  return constrain(voltage, -8.0f, 8.0f);
}

// 4. 辅助函数（用于获取米和米/秒）
float get_average_distance_meters()
{
  // 假设轮半径 r = 0.026m
  return -((encoderL.getAngle() + encoderR.getAngle()) / 2.0f) * 0.026f;
}

float get_average_velocity_mps()
{
  return -((motorL.shaftVelocity() + motorR.shaftVelocity()) / 2.0f) * 0.026f;
}
