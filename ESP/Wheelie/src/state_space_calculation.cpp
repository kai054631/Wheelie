#include "config.h"

Profile profile_list[] = {
    { 25, -0.23f,  -8.0f, -15.0f, 52.07f, 4.00f, 2.0f, 0.343f },
    { 45, -0.16f,  -8.0f, -12.0f, 56.60f, 4.00f, 2.0f, 0.343f },
    { 65, -0.10f, -10.0f, -15.0f, 59.79f, 2.77f, 2.0f, 0.343f },
    { 85, -0.07f,  -8.0f, -13.0f, 61.00f, 4.00f, 2.0f, 0.343f },
};
const int N_PROFILES = sizeof(profile_list) / sizeof(profile_list[0]);

float average_speed = 0;
float x1 = 0.0f;
float x2 = 0.0f;
float x3 = 0.0f;
float x4 = 0.0f;
float position_offset = 0.0f;

float K1 = profile_list[0].K1;
float K2 = profile_list[0].K2;
float K3 = profile_list[0].K3;
float K4 = profile_list[0].K4;
float K5 = profile_list[0].K5;
float K6 = profile_list[0].K6;
float angle_offset = profile_list[0].angle_offset;
int   Servo_angle  = profile_list[0].servo_angle;

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

//yaw angle control
