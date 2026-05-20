#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include "MyServo.h"
// #include <ESPAsyncWebServer.h>
// #include <WiFi.h>

// --- 引脚定义 ---
// mpu6050 pin
#define I2C_SDA 2
#define I2C_SCL 1
//servo pin
#define SERVO_L_PIN 4
#define SERVO_R_PIN 5

// --- 全局变量声明 (extern 表示实现在其他地方) ---
extern float angle_offset;
extern volatile float Pitch_Radian, Pitch_gyro;

extern int Servo_angle;
//state_space_variable
struct RobotState
{
     float position;
     float velocity;
     float pitch_angle;
     float gyro_rate;
};

extern const float K1; // 轮子水平位置反馈 (Position)
extern const float K2;  // 轮子水平速度反馈 (Velocity)
extern const float K3;   // 车身倾斜角度反馈 (Pitch Angle in Rad)
extern const float K4;   // 车身陀螺仪角速度反馈 (Gyro Rate in Rad/s)

extern float x1; // error in position, in meters
extern float x2; // error in velocity, in meters per second
extern float x3; // error in pitch angle, in radians
extern float x4; // error in gyro rate, in radians per second

//--- auto calibrate angle offset for the pitch angle ---
extern float average_speed;   //in m/s

// --- 硬件对象声明 ---
//mpu6050
extern MPU6050 mpu;
//2804 bldc motor
extern BLDCMotor motorL, motorR;
extern BLDCDriver3PWM driverL, driverR;
extern Encoder encoderL, encoderR;
//servo
extern MyServo LeftServo, RightServo;
//web server
// extern AsyncWebServer server;
// extern AsyncWebSocket ws; // 添加这一行

// --- 函数原型 ---
//kalman filter update function
float kalmanUpdate(float newAngle, float newRate, float dt);
//encoder interrupt handlers
void doLA(); 
void doLB(); 
void doRA(); 
void doRB();

// //web server setup function
// void setupWebServer();

float compute_LQR_balancing_voltage(RobotState current, RobotState target, float angle_offset_deg);
float get_average_distance_meters();
float get_average_velocity_mps();

#endif