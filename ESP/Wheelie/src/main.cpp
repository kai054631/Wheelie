#include "config.h"

// --- 变量定义 ---
// float angle_offset = -0.22; // in radians — tune via web UI to stop drift
volatile float Pitch_angle = 0.0, Pitch_gyro = 0.0;
volatile float shared_motor_voltage = 0.0; // share variable for taskbalance and taskmotor

//for 25 degree 2500mah
int Servo_angle = 25; 
float angle_offset = -0.232;
float K1 = -0.3162f; // 轮子水平位置反馈 (Position)
float K2 = -3.5285f; // 轮子水平速度反馈 (Velocity)
float K3 = 15.6848f; // 车身倾斜角度反馈 (Pitch Angle in Rad)
float K4 = 2.2882f;  // 车身陀螺仪角速度

// // // //for 45 degree
// int Servo_angle = 45;      
// float angle_offset = -0.17;
// float K1 = -10.0f;       // 轮子水平位置反馈 — disabled until pitch stable
// float K2 = -2.0f;       // 轮子水平速度反馈 — disabled until pitch stable
// float K3 = 75.0f;      // 车身倾斜角度反馈 — saturates at 6.5°, safe for startup
// float K4 = 1.5f;       // 车身陀螺仪角速度

RobotState currentState;
RobotState target = {0.0f, 0.0f, 0.0f, 0.0f};           // 目标状态：位置0，速度0，倾斜角0，陀螺仪角速度0
// --- 硬件对象定义 ---2.2052531
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
  delay(2000); // 等待系统稳定
  motorL.initFOC();
  motorR.initFOC();
  // printf("Motor L Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorL.zero_electric_angle, motorL.sensor_direction);
  // printf("Motor R Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorR.zero_electric_angle, motorR.sensor_direction);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    if (abs(currentState.pitch_angle-angle_offset) > 0.4f)
    {
      // Only call disable if the motor is currently running
      if (motorL.enabled)
      {
        motorL.disable();
        motorR.disable();
      }
    }
    else
    {
      // Execute the space-vector phase updates at 1kHz
      motorL.loopFOC();
      motorR.loopFOC();
      // ONLY enable if they were previously cut off by a fall event
      if (!motorL.enabled)
      {
        motorL.enable();
        motorR.enable();
      }
      float v = -shared_motor_voltage;
      motorL.move(v);
      motorR.move(v);
    }
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
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    Pitch_angle = kalmanUpdate(mpu.getAngleY(), mpu.getGyroY(), dt);
    // Low-pass filter gyro (α=0.25, τ≈6ms) — suppresses vibration noise fed into K4
    static float gyro_filtered = 0.0f;
    gyro_filtered = 0.08f * mpu.getGyroY() + 0.92f * gyro_filtered;
    Pitch_gyro = gyro_filtered;

    currentState.position = get_average_distance_meters();
    currentState.velocity = get_average_velocity_mps();
    currentState.pitch_angle = Pitch_angle * (PI / 180.0f); // in radian/s
    currentState.gyro_rate = Pitch_gyro * (PI / 180.0f);    // in rad/s
    
    // 使用你新定义的函数计算电压
    shared_motor_voltage = compute_LQR_balancing_voltage(currentState, target, angle_offset);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// --- 任务：Servo 控制 (Core 0) ---
void TaskServoCode(void *pvParameters)
{
  for (;;)
  {
     int servo_angle = constrain(Servo_angle, 20, 100);
        LeftServo.write(servo_angle);
        RightServo.write(servo_angle);
        vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --- 任务：监控 (Core 0) ---
void TaskMonitorCode(void *pv)
{
  char buffer[256];
  for (;;)
  {
    // Serial.printf("Motor_Speed: %.2f | Voltage:%.2f | Pitch Angle: %.2f\n", motorL.shaftVelocity(), shared_motor_voltage, Pitch_angle);
    // Serial.printf(" x1: %.2f, x2: %.2f ,x3: %.2f ,x4: %.2f , Pitch_Angle: %.2f , Voltage: %.2f , Left_Velocity: %.2f , Right_Velocity: %.2f, Motor_L_Angle: %.2f, Motor_R_Angle: %.2f\n", K1 * x1, K2 * x2, K3 * x3, K4 * x4, currentState.pitch_angle, shared_motor_voltage, motorL.shaftVelocity(), motorR.shaftVelocity(), encoderL.getAngle(), encoderR.getAngle());
    snprintf(buffer, sizeof(buffer),
             "meter_error:%.2f| x1: %.2f, x2: %.2f, x3: %.2f, x4: %.2f, Pitch_Angle: %.2f, Voltage: %.2f, Left_Velocity: %.2f, Right_Velocity: %.2f, Motor_L_Angle: %.2f, Motor_R_Angle: %.2f, Temperature: %.2f\n",
             x1,K1*x1, K2 * x2, K3 * x3, K4 * x4, 
             currentState.pitch_angle, -shared_motor_voltage, 
             motorL.shaftVelocity(), motorR.shaftVelocity(), 
             encoderL.getAngle(), encoderR.getAngle()
             ,mpu.getTemp());
    // snprintf(buffer,sizeof(buffer)," position: %.2f | target:%.2f \n",currentState.position,target.position);
    ws.cleanupClients();
    ws.textAll(buffer);
    Serial.print(buffer);

    vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms更新一次网页监视器
  }
}

void setup()
{
  Serial.begin(115200);
  // MPU6050
  // delay(2000); // 等待MPU6050上电稳定
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  mpu.begin();
  mpu.setGyroOffsets(-1.484977, -0.755755, -2.337404); // old values — board position changed
  mpu.setAccOffsets(-0.003804, -0.074246, 0.216630);
  // mpu.calcOffsets(true, true); // recalibrate for new battery spacer position
  // printf("Gyro X offset: %f\n", mpu.getGyroXoffset());
  // printf("Gyro Y offset: %f\n", mpu.getGyroYoffset());
  // printf("Gyro Z offset: %f\n", mpu.getGyroZoffset());
  // printf("Acc X offset: %f\n", mpu.getAccXoffset());
  // printf("Acc Y offset: %f\n", mpu.getAccYoffset());
  // printf("Acc Z offset: %f\n", mpu.getAccZoffset());

  setupWebServer();
  // 电机基础初始化 (不包含阻塞的 initFOC)
  encoderL.init();
  encoderL.enableInterrupts(doLA, doLB);
  driverL.voltage_power_supply = 12;
  motorL.voltage_limit = 8;
  driverL.init();
  motorL.linkSensor(&encoderL);
  motorL.linkDriver(&driverL);
  motorL.voltage_sensor_align = 5;
  motorL.controller = MotionControlType::torque;
  motorL.init();

  encoderR.init();
  encoderR.enableInterrupts(doRA, doRB);
  driverR.voltage_power_supply = 12;
  motorR.voltage_limit = 8;
  driverR.init();
  motorR.linkSensor(&encoderR);
  motorR.linkDriver(&driverR);
  motorR.controller = MotionControlType::torque;
  motorR.voltage_sensor_align = 5;
  motorR.init();
  // runMotor = true;

  // Servo
  LeftServo.setup(SERVO_L_PIN, 2000, 1000);
  RightServo.setup(SERVO_R_PIN, 1000, 2000);

  // 创建任务 (注意优先级：电机最高)
  xTaskCreatePinnedToCore(TaskMotorCode, "MotorTask", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskBalanceCode, "BalanceTask", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMonitorCode, "MonitorTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskServoCode, "ServoTask", 5000, NULL, 1, NULL, 0);
}
void loop() {}