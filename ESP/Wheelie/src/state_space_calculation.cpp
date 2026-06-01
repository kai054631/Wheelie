#include "config.h"

float average_speed = 0;
float x1 = 0.0f;
float x2 = 0.0f;
float x3 = 0.0f;
float x4 = 0.0f;
float position_offset = 0.0f;

// --- yaw state ---
float yaw_angle   = 0.0f;
float yaw_rate    = 0.0f;
float yaw_ref     = 0.0f;
bool  yaw_enabled = true;

const float YAW_DV_LIMIT = 2.0f;

float compute_LQR_balancing_voltage(RobotState current, RobotState target, float angle_offset_deg)
{
  float angle_offset_rad = angle_offset_deg;

  const float tau = 20.0f;
  const float dt  = 0.005f;
  position_offset += (current.position - target.position - position_offset) * (dt / tau);

  x1 = current.position - position_offset - target.position;
  x2 = current.velocity - target.velocity;
  x3 = current.pitch_angle - angle_offset_rad - target.pitch_angle;
  x4 = current.gyro_rate - target.gyro_rate;

  float voltage = -((K1 * x1) + (K2 * x2) + (K3 * x3) + (K4 * x4));
  return constrain(voltage, -4.0f, 4.0f);
}

// dv > 0 => right wheel faster => turns left (CCW from above)
// if turn direction is wrong, negate dv in TaskMotorCode
float compute_yaw_voltage(float psi, float psi_dot, float psi_ref)
{
  if (!yaw_enabled) return 0.0f;
  float e  = psi - psi_ref;
  float dv = -(K5 * e + K6 * psi_dot);
  return constrain(dv, -YAW_DV_LIMIT, YAW_DV_LIMIT);
}

float get_average_distance_meters()
{
  return -((encoderL.getAngle() + encoderR.getAngle()) / 2.0f) * WHEEL_RADIUS_M;
}

float get_average_velocity_mps()
{
  return -((motorL.shaftVelocity() + motorR.shaftVelocity()) / 2.0f) * WHEEL_RADIUS_M;
}
