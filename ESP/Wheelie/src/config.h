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

// --- 全局变量声明 (extern 表示实现在其他地方) ---
extern float Kp, Ki, Kd;
extern float angle_offset;
extern volatile float Pitch_angle, Pitch_gyro;
extern float output_voltage;
extern int Servo_angle;

extern float move_velocity;  // 前进后退速度偏移
extern float turn_velocity;  // 转向速度偏移
// --- 硬件对象声明 ---
extern MPU6050 mpu;

extern BLDCMotor motorL, motorR;
extern BLDCDriver3PWM driverL, driverR;
extern Encoder encoderL, encoderR;

extern MyServo LeftServo, RightServo;

extern AsyncWebServer server;
extern AsyncWebSocket ws; // 添加这一行

// --- 函数原型 ---
float kalmanUpdate(float newAngle, float newRate, float dt);

void doLA(); 
void doLB(); 
void doRA(); 
void doRB();

void setupWebServer();

#endif