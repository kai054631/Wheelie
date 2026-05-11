#include "config.h"

// --- 变量定义 ---
// float Kp = 2.6, Ki = 0.6, Kd = 0.07; // for servo angle =45
// float angle_offset = -6.0;  // for servo angle =45
// int Servo_angle = 45;

float Kp = 5.3, Ki = 0.02, Kd = 0.05; // for servo angle =25
float angle_offset = -9.20;           // for servo angle =25
int Servo_angle = 25;

volatile float Pitch_angle = 0.0, Pitch_gyro = 0.0;
float output_voltage = 0.0;
float move_velocity = 0; // in voltage
float turn_velocity = 0; // in voltage

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

// --- 任务：电机控制 (Core 1) ---
void TaskMotorCode(void *pv)
{
  // 关键：在任务内进行最后的初始化，防止 Core 0 和 Core 1 争抢电机
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  motorL.initFOC();
  motorR.initFOC();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    if (abs(Pitch_angle) < 20.0)
    {
      motorL.enable();
      motorR.enable();

      float target_angle = 0.0 + move_velocity;
      float error = target_angle - Pitch_angle;
      output_voltage = constrain((Kp * error) - (Kd * Pitch_gyro), -5.0, 5.0);
      motorL.loopFOC();
      motorR.loopFOC();
      motorL.move(-output_voltage + turn_velocity);
      motorR.move(-output_voltage - turn_velocity);
    }
    else
    {
      motorL.disable();
      motorR.disable();
    }
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
      LeftServo.speedControl(180 - servo_angle,5.0); // 速度控制，参数可调
      RightServo.speedControl(servo_angle,15.0);
    }
    else
    {
      Servo_angle = 100; // 限制最大角度
      // printf("Servo Angle: %d\n", Servo_angle);
      LeftServo.write(180 - Servo_angle);
      RightServo.write(Servo_angle);
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
    printf("Motor L: %.2f | Motor R: %.2f | Pitch: %.2f |Turn_Velocity: %.2f | Move_Velocity: %.2f | angle_offset: %.2f\n ", motorL.shaft_velocity, motorR.shaft_velocity, Pitch_angle, turn_velocity, move_velocity, angle_offset);

    vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms更新一次网页监视器
  }
}

void setup()
{
  printf("starting setup...\n");
  Serial.begin(115200);
  delay(1000);

  // MPU6050
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  mpu.begin();
  // mpu.calcOffsets(true, true);
  // printf("MPU6050 initialized with angle offset: %.2f\n", angle_offset);

  setupWebServer();
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // 电机基础初始化 (不包含阻塞的 initFOC)
  encoderL.init();
  encoderL.enableInterrupts(doLA, doLB);
  driverL.voltage_power_supply = 12;
  driverL.init();
  motorL.linkSensor(&encoderL);
  motorL.linkDriver(&driverL);
  motorL.voltage_sensor_align = 3;
  motorL.controller = MotionControlType::torque;
  motorL.init();
  if (motorL.initFOC() != 1)
  {
    printf("CRITICAL ERROR: Motor L FOC calibration FAILED!\n");
    motorL.disable();
    while (1)
      ;
  }
  printf("Motor L FOC success.\n");

  encoderR.init();
  encoderR.enableInterrupts(doRA, doRB);
  driverR.voltage_power_supply = 12;
  driverR.init();
  motorR.linkSensor(&encoderR);
  motorR.linkDriver(&driverR);
  motorR.controller = MotionControlType::torque;
  motorR.voltage_sensor_align = 3;
  motorR.init();
  if (motorR.initFOC() != 1)
  {
    printf("CRITICAL ERROR: Motor R FOC calibration FAILED!\n");
    motorR.disable();
    while (1)
      ;
  }
  printf("Motor R FOC success. System ready.\n");

  // Servo
  LeftServo.setup(SERVO_L_PIN, 1000, 2000);
  RightServo.setup(SERVO_R_PIN, 1000, 2200);

  // 创建任务 (注意优先级：电机最高)
  xTaskCreatePinnedToCore(TaskMotorCode, "MotorTask", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskBalanceCode, "BalanceTask", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMonitorCode, "MonitorTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskServoCode, "ServoTask", 5000, NULL, 1, NULL, 0);
}

void loop() {}