#include "config.h"

// --- 变量定义 ---
float Kp = 4.0, Ki = 0.09, Kd = 0.05; // for servo angle =25
float angle_offset = -9.2;            // for servo angle =25
volatile float Pitch_angle = 0.0, Pitch_gyro = 0.0;
float Pitch_integral = 0.0;

float output_voltage = 0.0;
float move_velocity = 0; // in voltage
int Servo_angle = 25;

float YawKp = 0.05, YawKi = 0.0, YawKd = 0.01; // for servo angle =25
float turn_velocity = 0;                     // in voltage
volatile float Yaw_angle = 0.0, Pre_yaw_angle=0.0, Yaw_gyro = 0.0;

// --- 硬件对象定义 ---
MPU6050 mpu(Wire);
BLDCMotor motorL(7), motorR(7);
BLDCDriver3PWM driverL(14, 13, 12, 11), driverR(16, 15, 7, 6);
Encoder encoderL(3, 46, 1024), encoderR(17, 18, 1024);
MyServo LeftServo("LeftServo"), RightServo("RightServo");

// --- 编码器中断函数 ---
void doLA() { encoderL.handleA(); }
void doLB() { encoderL.handleB(); }
void doRA() { encoderR.handleA(); }
void doRB() { encoderR.handleB(); }

//--- auto calibrate angle offset for the pitch angle ---
float average_speed = 0;
float calibration_step = 0.001; // How fast to adjust the angle
bool is_calibrating = false;

void TaskAutoCalibrate(void *pvParameters)
{
  for (;;)
  {
    if (is_calibrating)
    {
      if (abs(Pitch_angle) < 10.0)
      {
        // 1. Get the current motor speed (using SimpleFOC's velocity or your PID output)
        // We use a simple Low Pass Filter to smooth out the noisy speed data
        float current_speed = (motorL.shaft_velocity + motorR.shaft_velocity) / 2.0;
        printf("current_speed: %f\n", current_speed);
        average_speed = (average_speed * 0.95) + (current_speed * 0.05);

        // 2. Adjust the offset based on the speed drift
        if (average_speed > 0.1)
        {
          // Drifting forward: Lean it backward slightly
          angle_offset += calibration_step;
        }
        else if (average_speed < -0.1)
        {
          // Drifting backward: Lean it forward slightly
          angle_offset -= calibration_step;
        }
        else if (abs(average_speed) < 0.05)
        {
          // Speed is practically zero! We found the sweet spot.
          printf("Calibration Complete! New Angle Offset: %f\n", angle_offset);
          is_calibrating = false; // Stop calibrating
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Run every 100ms
  }
}
// --- 任务：电机控制 (Core 1) ---
void TaskMotorCode(void *pv)
{
  // 关键：在任务内进行最后的初始化，防止 Core 0 和 Core 1 争抢电机
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  if (motorL.initFOC() != 1)
  {
    printf("CRITICAL ERROR: Motor L FOC calibration FAILED!\n");
    motorL.disable();
    while (1)
      ;
  }
  printf("Motor L FOC success.\n");
  if (motorR.initFOC() != 1)
  {
    printf("CRITICAL ERROR: Motor R FOC calibration FAILED!\n");
    motorR.disable();
    while (1)
      ;
  }
  printf("Motor R FOC success. System ready.\n");
  // is_calibrating = true;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    if (abs(Pitch_angle) < 10.0)
    {
      motorL.enable();
      motorR.enable();

      float target_angle = 0.0 + move_velocity;
      
      float error = target_angle - Pitch_angle;
      Pitch_integral += error; 
      Pitch_integral = constrain(Pitch_integral, -150.0, 150.0);
      Pre_yaw_angle = (0.15*Pre_yaw_angle) + (0.85*Yaw_angle);
      float error_yaw = 0.0 - Yaw_angle; // 目标是保持 Yaw_angle 在 0
      turn_velocity = constrain((YawKp * error_yaw) - (YawKd * Yaw_gyro), -3.0, 3.0);

      output_voltage = constrain((Kp * error) +(Ki * Pitch_integral)- (Kd * Pitch_gyro), -6.0, 6.0);
      motorL.loopFOC();
      motorR.loopFOC();
      if (abs(turn_velocity) > 1)
      {
        motorL.move(-output_voltage + turn_velocity);
        motorR.move(-output_voltage - turn_velocity);
      }
      else
      {
        motorL.move(-output_voltage + 0);
        motorR.move(-output_voltage - 0);
      }
    }
    else
    {
      motorL.disable();
      motorR.disable();
    }
    Pre_yaw_angle = Yaw_angle;
    // 1ms 周期
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

// --- 任务：姿态计算 (Core 0) ---
void TaskBalanceCode(void *pv)
{
  unsigned long lastTime = millis();
  for (;;)
  {
    mpu.update();
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    Yaw_angle = 0.98 * (Yaw_angle + mpu.getGyroZ() * dt) + 0.02 * mpu.getAngleZ();
    Yaw_gyro = mpu.getGyroZ();
    Pitch_angle = kalmanUpdate(mpu.getAngleY(), mpu.getGyroY(), dt) - angle_offset;
    Pitch_gyro = mpu.getGyroY();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// --- 任务：Servo 控制 (Core 0) ---
void TaskServoCode(void *pvParameters)
{
  for (;;)
  {
    if (Servo_angle <= 100)
    {
      int servo_angle = constrain(Servo_angle, 20, 100); // 直接使用全局变量，避免重复读取
      // printf("Servo Angle: %d\n", Servo_angle);
      LeftServo.write(180 - servo_angle); // 速度控制，参数可调
      RightServo.write(servo_angle);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --- 任务：监控 (Core 0) ---
void TaskMonitorCode(void *pv)
{
  for (;;)
  {
    // 构造数据字符串: "角度,电压"
    String data = String(Pitch_angle, 2) + "," + String(output_voltage, 2);

    // 通过 WebSocket 发送给所有连接的网页客户端
    ws.textAll(data);
    // 同时保留串口输出方便电脑查看
    // Serial.println(data);
    // printf("Motor L: %.2f | Motor R: %.2f | Pitch: %.2f |Turn_Velocity: %.2f | Move_Velocity: %.2f | angle_offset: %.2f\n ", motorL.shaft_velocity, motorR.shaft_velocity, Pitch_angle, turn_velocity, move_velocity, angle_offset);
    Serial.printf("Yaw: %.2f |X_Angle:%.2f | y: %.2f | z: %.2f \n", Yaw_angle, turn_velocity, Pitch_angle, mpu.getAngleZ());

    vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms更新一次网页监视器
  }
}

void setup()
{
  Serial.begin(115200);

  // MPU6050
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  mpu.begin();
  // mpu.calcOffsets(true, true);
  // printf("MPU6050 initialized with angle offset: %.2f\n", angle_offset);

  // setupWebServer();
  // vTaskDelay(2000 / portTICK_PERIOD_MS);

  // 电机基础初始化 (不包含阻塞的 initFOC)
  encoderL.init();
  encoderL.enableInterrupts(doLA, doLB);
  driverL.voltage_power_supply = 12;
  driverL.init();
  motorL.linkSensor(&encoderL);
  motorL.linkDriver(&driverL);
  motorL.voltage_sensor_align = 5;
  motorL.controller = MotionControlType::torque;
  motorL.init();

  encoderR.init();
  encoderR.enableInterrupts(doRA, doRB);
  driverR.voltage_power_supply = 12;
  driverR.init();
  motorR.linkSensor(&encoderR);
  motorR.linkDriver(&driverR);
  motorR.controller = MotionControlType::torque;
  motorR.voltage_sensor_align = 5;
  motorR.init();

  // Servo
  LeftServo.setup(SERVO_L_PIN, 1000, 2000);
  RightServo.setup(SERVO_R_PIN, 1000, 2000);

  // 创建任务 (注意优先级：电机最高)
  xTaskCreatePinnedToCore(TaskMotorCode, "MotorTask", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskBalanceCode, "BalanceTask", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMonitorCode, "MonitorTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskServoCode, "ServoTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskAutoCalibrate, "AutoCalTask", 5000, NULL, 1, NULL, 0);
}
void loop() {}