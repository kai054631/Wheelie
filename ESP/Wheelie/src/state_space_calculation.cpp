#include "config.h"

Profile profile_list[] = {
    {25, -0.27f, -8.0f, -10.0f, 52.07f, 2.40f, 1.5f, 0.343f},
    {45, -0.19f, -8.0f, -12.0f, 56.60f, 2.4f, 1.5f, 0.343f},
    {65, -0.12f, -10.0f, -15.0f, 59.79f, 2.77f, 1.5f, 0.343f},
    {85, -0.11f, -8.0f, -13.0f, 61.00f, 4.00f, 1.5f, 0.343f},
};
const int N_PROFILES = sizeof(profile_list) / sizeof(profile_list[0]);

float average_speed = 0;
float x1 = 0.0f;
float x2 = 0.0f;
float x3 = 0.0f;
float x4 = 0.0f;
float position_offset = 0.0f;
float pos_setpoint = 0.0f;
float vel_ff_ramp = 0.0f;   // actual ramped feedforward velocity (monitored)

float K1 = profile_list[0].K1;
float K2 = profile_list[0].K2;
float K3 = profile_list[0].K3;
float K4 = profile_list[0].K4;
float K5 = profile_list[0].K5;
float K6 = profile_list[0].K6;
float rad_offset = profile_list[0].rad_offset;
int Servo_angle = profile_list[0].servo_angle;

// --- yaw state ---
float yaw_ref = 0.0f;
bool yaw_enabled = true;
float yaw_e = 0.0f;
float yaw_differ = 0.0f;
float yaw_mpu = 0.0f;

const float YAW_DV_LIMIT = 2.0f;

float compute_LQR_balancing_voltage(RobotState current, RobotState target, float angle_offset_deg)
{
  float rad_offset_rad = rad_offset;
  const float dt = 0.005f;
  const float DEFAULT_SPEED = 0.15f;
  const float MAX_ACCEL = 0.4f;   // m/s² — limits how fast vel_ff_ramp changes

  // Traversal speed = commanded target velocity magnitude (or a gentle default).
  // NOTE: the yaw channel is fully decoupled (added later as ±dv), so we do NOT
  // slow forward motion while turning — that previously made the robot "stop and
  // turn". Forward speed is now held; the ±2 V dv clamp protects the balancer.
  float trav_speed = (fabs(target.velocity) > 0.005f) ? fabs(target.velocity) : DEFAULT_SPEED;

  // Trapezoidal profile: decelerate early enough to stop at target
  float pos_err = target.position - pos_setpoint;
  float decel_dist = (vel_ff_ramp * vel_ff_ramp) / (2.0f * MAX_ACCEL);
  float desired_vel;
  if (fabs(pos_err) < 0.005f) {
    desired_vel = 0.0f;
    pos_setpoint = target.position;
  } else if (fabs(pos_err) <= decel_dist * 1.1f) {
    desired_vel = 0.0f;   // inside braking zone — ramp down to stop
  } else {
    desired_vel = (pos_err > 0) ? trav_speed : -trav_speed;
  }

  // Accelerate/decelerate vel_ff_ramp toward desired_vel
  float vel_step = MAX_ACCEL * dt;
  if (fabs(desired_vel - vel_ff_ramp) <= vel_step) {
    vel_ff_ramp = desired_vel;
  } else {
    vel_ff_ramp += (desired_vel > vel_ff_ramp) ? vel_step : -vel_step;
  }
  pos_setpoint += vel_ff_ramp * dt;

  // Slow integrator — corrects steady-state position drift over ~20 s
  const float tau = 40.0f;
  position_offset += (current.position - pos_setpoint - position_offset) * (dt / tau);

  x1 = current.position - pos_setpoint - position_offset;
  current.velocity = constrain(current.velocity, -0.8f, 0.8f); // for monitoring only; sign matches shaftVelocity()
  x2 = current.velocity - vel_ff_ramp;
  x3 = current.pitch_rad - rad_offset_rad - target.pitch_rad;
  x4 = current.gyro_rate - target.gyro_rate;

  float voltage = -((K1 * x1) + (K2 * x2) + (K3 * x3) + (K4 * x4));
  return constrain(voltage, -6.0f, 6.0f);
}

float compute_yaw_voltage(float psi, float psi_dot, float psi_ref)
{
  if (!yaw_enabled)
    return 0.0f;
  yaw_mpu = mpu.getGyroZ() * (PI / 180.0f); // monitoring only; gyro-Z fusion not used (see handoff §4.4)
  yaw_e = psi - psi_ref;
  float dv = -(K5 * yaw_e + K6 * psi_dot); // K5 = yaw-angle gain, K6 = yaw-rate gain

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
