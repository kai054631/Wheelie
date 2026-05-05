#include <Arduino.h>
#include "MyServo.h"
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>
// --- 定义任务句柄 ---
TaskHandle_t TaskMotor;
TaskHandle_t TaskMonitor;
// TaskHandle_t TaskBalance;

// --- MPU6050 ---
#define I2C_SDA 2
#define I2C_SCL 1
MPU6050 mpu(Wire);

// --- 卡尔曼滤波变量 ---
float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.03;
float kalmanAngle = 0, bias = 0;
float P[2][2] = {{1, 0}, {0, 1}};

// --- SimpleFOC 驱动与编码器 ---
BLDCMotor motorL = BLDCMotor(7);
BLDCDriver3PWM driverL = BLDCDriver3PWM(14, 13, 12, 11);
Encoder encoderL = Encoder(3, 46, 1024);
void doLA() { encoderL.handleA(); }
void doLB() { encoderL.handleB(); }

BLDCMotor motorR = BLDCMotor(7);
BLDCDriver3PWM driverR = BLDCDriver3PWM(16, 15, 7, 6);
Encoder encoderR = Encoder(17, 18, 1024);
void doRA() { encoderR.handleA(); }
void doRB() { encoderR.handleB(); }

MyServo LeftServo("LeftServo");
MyServo RightServo("RightServo");

// --- PID 与控制变量 ---
float Kp = -0.6, Ki = -0.0, Kd = -0.05;
volatile float Pitch_angle = 0.0;
volatile float Pitch_gyro = 0.0;
float output_voltage = 0.0;
float angle_offset = -2.0;

// --- 卡尔曼计算函数 ---
float kalmanUpdate(float newAngle, float newRate, float dt)
{
  float rate = newRate - bias;
  kalmanAngle += dt * rate;
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyro * dt;
  float S = P[0][0] + R_angle;
  float K[2] = {P[0][0] / S, P[1][0] / S};
  float y = newAngle - kalmanAngle;
  kalmanAngle += K[0] * y;
  bias += K[1] * y;
  float P00_temp = P[0][0], P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  return kalmanAngle;
}

// --- 任务：电机控制 ---
void TaskMotorCode(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1; // 1ms 周期 (1kHz)
  unsigned long lastTime = millis();
  for (;;)
  {
    mpu.update();
    unsigned long now = millis();
    float error = 0.0 - Pitch_angle;
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    Pitch_angle = kalmanUpdate(mpu.getAngleY(), mpu.getGyroY(), dt) - angle_offset;
    Pitch_gyro = mpu.getGyroY();

    if (abs(Pitch_angle) < 15.0) // 仅在角度较小时控制电机，避免过大输出
    {
      motorL.enable();
      motorR.enable();
      error = 0.0 - Pitch_angle;
      output_voltage = (Kp * error) - (Kd * Pitch_gyro);
      output_voltage = constrain(output_voltage, -5.0, 5.0);
      motorL.loopFOC();
      motorR.loopFOC();
      motorL.move(output_voltage);
      motorR.move(output_voltage);
    }
    else
    {
      motorL.disable();
      motorR.disable();
      output_voltage = 0;
      error = 0;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// --- 任务：姿态读取 ---
// void TaskBalanceCode(void *pvParameters)
// {
//   unsigned long lastTime = millis();
//   for (;;)
//   {
//   }
// }

// --- 任务：监控输出 ---
void TaskMonitorCode(void *pvParameters)
{
  for (;;)
  {
    printf("%f,%f,%f\n", output_voltage, Pitch_angle, mpu.getGyroY());
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  mpu.begin();
  // mpu.calcOffsets(true, true);

  encoderL.init();
  encoderL.enableInterrupts(doLA, doLB);
  motorL.linkSensor(&encoderL);
  driverL.voltage_power_supply = 12;
  driverL.init();
  motorL.linkDriver(&driverL);
  motorL.controller = MotionControlType::torque;
  motorL.init();
  motorL.initFOC();

  encoderR.init();
  encoderR.enableInterrupts(doRA, doRB);
  motorR.linkSensor(&encoderR);
  driverR.voltage_power_supply = 12;
  driverR.init();
  motorR.linkDriver(&driverR);
  motorR.controller = MotionControlType::torque;
  motorR.init();
  motorR.initFOC();

  xTaskCreatePinnedToCore(TaskMotorCode, "MotorTask", 10000, NULL, 3, &TaskMotor, 1);
  xTaskCreatePinnedToCore(TaskMonitorCode, "MonitorTask", 10000, NULL, 0, &TaskMonitor, 0);
  // xTaskCreatePinnedToCore(TaskBalanceCode, "BalanceTask", 10000, NULL, 2, &TaskBalance, 0);
}

void loop()
{
  // 必须留空，但不能删除
}