#include "config.h"

// --- 变量定义 ---
volatile float Pitch_angle = 0.0, Pitch_gyro = 0.0;
volatile float shared_motor_voltage_L = 0.0;
volatile float shared_motor_voltage_R = 0.0;

struct profile
{
  int servo_angle;
  float angle_offset;
  float K1;
  float K2;
  float K3;
  float K4;
  float K5;
  float K6;
} profile_list[] =
    {
        {25, -0.2, -8, -15, 52.07, 4, 2, 0.343f},
        {45, -0.14, -8, -12, 56.6, 4, 2, 0.343f},
        {65, -0.07, -10, -15, 59.79, 2.77, 2, 0.343f},
        {85, 0.0, -8, -13, 61, 4, 2, 0.343f}}

;
RobotState currentState;
RobotState target = {0.0f, 0.0f, 0.0f, 0.0f}; // 目标状态：位置0，速度0，倾斜角0，陀螺仪角速度0

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
  delay(2000); // 等待系统稳定
  motorL.initFOC();
  motorR.initFOC();
  // printf("Motor L Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorL.zero_electric_angle, motorL.sensor_direction);
  // printf("Motor R Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorR.zero_electric_angle, motorR.sensor_direction);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    if (abs(currentState.pitch_angle - angle_offset) > 0.4f)
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
      motorL.move(-shared_motor_voltage_L);
      motorR.move(-shared_motor_voltage_R);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

// --- 任务：姿态计算 + 偏航控制 (Core 0) ---
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
    static float gyro_filtered = 0.0f;
    gyro_filtered = 0.95f * mpu.getGyroY() + 0.05f * gyro_filtered;
    Pitch_gyro = gyro_filtered;

    currentState.position = get_average_distance_meters();
    currentState.velocity = get_average_velocity_mps();
    currentState.pitch_angle = Pitch_angle * (PI / 180.0f);
    currentState.gyro_rate = Pitch_gyro * (PI / 180.0f);

    // yaw rate from wheel speed differential; positive => turning left (CCW from above)
    yaw_rate = (motorR.shaftVelocity() - motorL.shaftVelocity()) * WHEEL_RADIUS_M / WHEEL_BASE_M;
    yaw_angle += yaw_rate * dt;

    float v = compute_LQR_balancing_voltage(currentState, target, angle_offset);
    float dv = compute_yaw_voltage(yaw_angle, yaw_rate, yaw_ref);

    // u_L = v - dv,  u_R = v + dv
    shared_motor_voltage_L = v + dv;
    shared_motor_voltage_R = v - dv;

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// --- 任务：Servo 控制 (Core 0) ---
void TaskServoCode(void *pvParameters)
{
  for (;;)
  {
    int servo_angle = constrain(Servo_angle, 20, 100);
    int right_trim = (int)roundf(0.06f * servo_angle - 1.3f);
    int right_angle = constrain(servo_angle + right_trim, 20, 100);
    LeftServo.write(180 - servo_angle);
    RightServo.write(right_angle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --- 任务：监控 (Core 0) ---
void TaskMonitorCode(void *pv)
{
  char buffer[320];
  for (;;)
  {
    snprintf(buffer, sizeof(buffer),
             "meter_error:%.2f x1:%.2f x2:%.2f x3:%.2f x4:%.2f Pitch_Angle:%.3f Voltage:%.2f VL:%.2f VR:%.2f yaw_angle:%.3f yaw_rate:%.3f Left_Velocity:%.2f Right_Velocity:%.2f Motor_L_Angle:%.2f Motor_R_Angle:%.2f Temp:%.1f\n",
             x1, K1 * x1, K2 * x2, K3 * x3, K4 * x4,
             currentState.pitch_angle,
             -(shared_motor_voltage_L + shared_motor_voltage_R) * 0.5f,
             -shared_motor_voltage_L, -shared_motor_voltage_R,
             yaw_angle, yaw_rate,
             motorL.shaftVelocity(), motorR.shaftVelocity(),
             encoderL.getAngle(), encoderR.getAngle(),
             mpu.getTemp());
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

  // Servo
  LeftServo.setup(SERVO_L_PIN, 1000, 2000);
  RightServo.setup(SERVO_R_PIN, 1000, 2000);

  // 创建任务 (注意优先级：电机最高)
  xTaskCreatePinnedToCore(TaskMotorCode, "MotorTask", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskBalanceCode, "BalanceTask", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMonitorCode, "MonitorTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskServoCode, "ServoTask", 5000, NULL, 1, NULL, 0);
}
void loop() {}