// =====================================================================
// six_state_yaw.cpp  --  reference integration for Wheelie
// 6-state LQR (longitudinal balancing) + yaw heading/turn controller
// =====================================================================
//
// This is a DROP-IN REFERENCE matching the architecture documented in
// HANDOFF.md (functions compute_LQR_balancing_voltage(), the gain table
// in state_space_calculation.cpp, the encoder->yaw_rate relation, and the
// motorL/R.move(-v) sign convention). Variable names follow the handoff;
// adjust to the exact identifiers in your config.h once the source is in.
//
// State vector (6):  x = [p, p_dot, theta, theta_dot, psi, psi_dot]
// Two channels:      v  = common-mode per-motor voltage  (balancing)
//                    dv = differential per-motor voltage (yaw)
// Apply:  u_R = v + dv ,  u_L = v - dv     (dv>0 => CCW / left turn)
// ---------------------------------------------------------------------

#include "config.h"
#include <math.h>

// ---- gains -----------------------------------------------------------
// K1..K4 : longitudinal (per gain-schedule row).  K5,K6 : yaw (shared).
// GAIN_TABLE rows are produced by lqr_compute_6state.py:
//   {servo_deg, K1, K2, K3, K4, K5, K6}
extern float K1, K2, K3, K4;          // active longitudinal gains
float K5 = 1.414f;                    // yaw angle gain   (psi)
float K6 = 0.343f;                    // yaw rate gain    (psi_dot)

// ---- yaw state -------------------------------------------------------
float yaw_angle    = 0.0f;            // psi  (rad), integrated from rate
float yaw_rate     = 0.0f;            // psi_dot (rad/s), from encoders
float yaw_ref      = 0.0f;            // heading setpoint (rad)
bool  yaw_enabled  = true;            // heading-hold on by default

const float YAW_DV_LIMIT = 2.0f;      // clamp differential volts (V)

// =====================================================================
// Longitudinal balancing voltage (unchanged 4-state law, kept separate)
// =====================================================================
float compute_LQR_balancing_voltage(float p, float p_dot,
                                     float theta, float theta_dot)
{
    float v = -(K1*p + K2*p_dot + K3*theta + K4*theta_dot);
    // existing +/-8 V clamp (HANDOFF 6.5)
    if (v >  8.0f) v =  8.0f;
    if (v < -8.0f) v = -8.0f;
    return v;
}

// =====================================================================
// Yaw differential voltage.  dv = -(K5*(psi-psi_ref) + K6*psi_dot)
// =====================================================================
float compute_yaw_voltage(float psi, float psi_dot, float psi_ref)
{
    if (!yaw_enabled) return 0.0f;
    float e  = psi - psi_ref;
    float dv = -(K5*e + K6*psi_dot);
    if (dv >  YAW_DV_LIMIT) dv =  YAW_DV_LIMIT;
    if (dv < -YAW_DV_LIMIT) dv = -YAW_DV_LIMIT;
    return dv;
}

// =====================================================================
// Inside TaskBalanceCode (500 Hz), AFTER reading encoders & IMU:
// =====================================================================
//
//   // --- yaw rate from the two wheel velocities (rad/s) ---
//   // shaftVelocity() is rad/s of the wheel; *R = m/s ground speed.
//   yaw_rate   = (motorR.shaftVelocity() - motorL.shaftVelocity()) * R / d;
//   yaw_angle += yaw_rate * dt;                 // dt = 1/500 s
//
//   float v  = compute_LQR_balancing_voltage(x1, x2, x3, x4);
//   float dv = compute_yaw_voltage(yaw_angle, yaw_rate, yaw_ref);
//
//   shared_motor_voltage_L = v - dv;
//   shared_motor_voltage_R = v + dv;
//
// Inside TaskMotorCode (1 kHz):
//   motorL.move(-shared_motor_voltage_L);
//   motorR.move(-shared_motor_voltage_R);
//
// NOTE: if a "turn left" command makes the robot turn right, flip the
// sign of dv in the two apply lines (single-line fix) -- this only
// depends on your encoder A/B wiring polarity.
// =====================================================================

// =====================================================================
// Web endpoints (extend webhandle.cpp /set handler):
//   /set?type=K5&val=X     update yaw angle gain
//   /set?type=K6&val=X     update yaw rate gain
//   /set?type=Y&val=X      set yaw_ref in DEGREES (turn command)
//   /set?type=YH&val=1|0   yaw heading-hold enable/disable
//
//   else if (type == "K5") K5 = val;
//   else if (type == "K6") K6 = val;
//   else if (type == "Y")  yaw_ref = val * (PI/180.0f);
//   else if (type == "YH") yaw_enabled = (val != 0.0f);
//
// /reset should also zero the heading:  yaw_angle = 0; yaw_ref = 0;
// =====================================================================
