#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include "MyServo.h"
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

#define I2C_SDA 2
#define I2C_SCL 1
#define SERVO_L_PIN 4
#define SERVO_R_PIN 5

#define WHEEL_RADIUS_M 0.026f   // wheel radius (m)
#define WHEEL_BASE_M   0.183f   // track width: distance between wheel contact points (m)

// --- Optional Xbox (BLE) controller ---------------------------------------
// Comment this out to build the firmware without controller support.
// #define USE_XBOX_CONTROLLER 1

extern float rad_offset;
extern volatile float Pitch_rad, Pitch_gyro;
extern volatile float shared_motor_voltage_L, shared_motor_voltage_R;
extern int Servo_angle;

struct Profile {
    int   servo_angle;
    float rad_offset;
    float K1, K2, K3, K4, K5, K6;
};

extern Profile profile_list[];
extern const int N_PROFILES;

struct RobotState
{
    float position;
    float velocity;
    float pitch_rad;
    float gyro_rate;
    float yaw_rad;     // psi (rad), integrated from wheel differential
    float yaw_rate;    // psi_dot (rad/s)
};

extern float K1;   // wheel position feedback
extern float K2;   // wheel velocity feedback
extern float K3;   // pitch angle feedback
extern float K4;   // pitch rate feedback
extern float K5;   // yaw angle gain   (psi)
extern float K6;   // yaw rate gain    (psi_dot)

extern float x1, x2, x3, x4;

extern float position_offset;
extern float pos_setpoint;
extern float vel_ff_ramp;
extern float pitch_comp;
extern RobotState currentState;
extern RobotState target;

// yaw state

extern float yaw_ref;     // heading setpoint (rad)
extern bool  yaw_enabled; // heading-hold on/off
extern float yaw_e;       // yaw angle error (psi - psi_ref)
extern float yaw_differ;  // raw wheel velocity difference (motorR - motorL, rad/s)
extern float yaw_mpu;     // MPU gyro-Z reading (rad/s) — monitoring only

extern float average_speed;

extern int  active_profile;
extern void applyProfile(int idx);

extern MPU6050 mpu;
extern BLDCMotor motorL, motorR;
extern BLDCDriver3PWM driverL, driverR;
extern Encoder encoderL, encoderR;
extern MyServo LeftServo, RightServo;
extern AsyncWebServer server;
extern AsyncWebSocket ws;

float kalmanUpdate(float newAngle, float newRate, float dt);
void doLA();
void doLB();
void doRA();
void doRB();
void setupWebServer();
float compute_LQR_balancing_voltage(RobotState current, RobotState target, float angle_offset_deg);
float compute_yaw_voltage(float psi, float psi_dot, float psi_ref);
float get_average_distance_meters();
float get_average_velocity_mps();

#ifdef USE_XBOX_CONTROLLER
void TaskControllerCode(void *pv);   // defined in controller.cpp
#endif

#endif
