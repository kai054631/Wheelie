#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include "MyServo.h"
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

// --- 引脚定义 ---
#define I2C_SDA 2
#define I2C_SCL 1
#define SERVO_L_PIN 4
#define SERVO_R_PIN 5

#define WHEEL_RADIUS_M 0.026f   // wheel radius (m)
#define WHEEL_BASE_M   0.16f    // distance between wheel contact points (m) — tune if yaw is reversed

// --- 全局变量声明 ---
extern float angle_offset;
extern volatile float Pitch_angle, Pitch_gyro;
extern volatile float shared_motor_voltage_L, shared_motor_voltage_R;
extern int Servo_angle;

struct Profile {
    int   servo_angle;
    float angle_offset;
    float K1, K2, K3, K4, K5, K6;
};

extern Profile profile_list[];
extern const int N_PROFILES;

struct RobotState
{
    float position;
    float velocity;
    float pitch_angle;
    float gyro_rate;
};

extern float K1;   // wheel position feedback
extern float K2;   // wheel velocity feedback
extern float K3;   // pitch angle feedback
extern float K4;   // pitch rate feedback
extern float K5;   // yaw angle gain   (psi)
extern float K6;   // yaw rate gain    (psi_dot)

extern float x1, x2, x3, x4;

extern float position_offset;
extern RobotState target;

// yaw state
extern float yaw_angle;    // psi (rad), integrated from wheel differential
extern float yaw_rate;     // psi_dot (rad/s)
extern float yaw_ref;      // heading setpoint (rad)
extern bool  yaw_enabled;  // heading-hold on/off

extern float average_speed;

extern int  active_profile;
extern void applyProfile(int idx);

// --- 硬件对象声明 ---
extern MPU6050 mpu;
extern BLDCMotor motorL, motorR;
extern BLDCDriver3PWM driverL, driverR;
extern Encoder encoderL, encoderR;
extern MyServo LeftServo, RightServo;
extern AsyncWebServer server;
extern AsyncWebSocket ws;

// --- 函数原型 ---
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

#endif
