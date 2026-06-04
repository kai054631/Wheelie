#include "config.h"

// --- 变量定义 ---
volatile float Pitch_rad = 0.0, Pitch_gyro = 0.0;
volatile float shared_motor_voltage_L = 0.0;
volatile float shared_motor_voltage_R = 0.0;

RobotState currentState;
RobotState target = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 目标状态：位置0，速度0，倾斜角0，陀螺仪角速度0

// --- 硬件对象定义 ---
MPU6050 mpu(Wire);
BLDCMotor motorL(7), motorR(7);
BLDCDriver3PWM driverL(14, 13, 12, 11), driverR(16, 15, 7, 6);
Encoder encoderL(3, 46, 1024), encoderR(17, 18, 1024);
MyServo LeftServo("LeftServo"), RightServo("RightServo");

int active_profile = 0;

void applyProfile(int idx) {
    if (idx < 0 || idx >= N_PROFILES) return;
    active_profile = idx;
    const Profile &p = profile_list[idx];
    Servo_angle = p.servo_angle;
    rad_offset  = p.rad_offset;
    K1 = p.K1;  K2 = p.K2;
    K3 = p.K3;  K4 = p.K4;
    K5 = p.K5;  K6 = p.K6;
    // Snap trajectory to current position so robot stays in place after switch
    float cur_pos   = get_average_distance_meters();
    pos_setpoint    = cur_pos;
    target.position = cur_pos;
    position_offset = 0.0f;
    target.velocity = 0.0f;
    yaw_ref = currentState.yaw_rad;
    Serial.printf("[profile] switched to %d — servo=%d° K3=%.2f K4=%.2f offset=%.3f\n",
                  idx, p.servo_angle, p.K3, p.K4, p.rad_offset);
}

// --- 编码器中断函数 ---
void doLA() { encoderL.handleA(); }
void doLB() { encoderL.handleB(); }
void doRA() { encoderR.handleA(); }
void doRB() { encoderR.handleB(); }

// --- 任务：电机控制 (Core 1) ---
void TaskMotorCode(void *pv)
{
  delay(1000); // 等待系统稳定
  motorL.initFOC();
  motorR.initFOC();
  // printf("Motor L Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorL.zero_electric_angle, motorL.sensor_direction);
  // printf("Motor R Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorR.zero_electric_angle, motorR.sensor_direction);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    bool fallen   = abs(currentState.pitch_rad - rad_offset) > 0.4f;
    bool airborne = (abs(motorL.shaftVelocity()) + abs(motorR.shaftVelocity())) > 40.0f
                    && abs(currentState.pitch_rad - rad_offset) < 0.08f;

    if (fallen)// || airborne)
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
    Pitch_rad = kalmanUpdate(mpu.getAngleY(), mpu.getGyroY(), dt);
    static float gyro_filtered = 0.0f;
    gyro_filtered = 0.95f * mpu.getGyroY() + 0.05f * gyro_filtered;
    Pitch_gyro = gyro_filtered;

    currentState.position = get_average_distance_meters();
    currentState.velocity = get_average_velocity_mps();
    currentState.pitch_rad = Pitch_rad * (PI / 180.0f);
    currentState.gyro_rate = Pitch_gyro * (PI / 180.0f);

    // yaw rate from wheel speed differential; positive => turning left (CCW from above)
    currentState.yaw_rate = (constrain((motorR.shaftVelocity() - motorL.shaftVelocity()) * WHEEL_RADIUS_M / WHEEL_BASE_M,-6.28,6.28))
    ; // combine wheel-based yaw rate with gyro Z for better accuracy
    currentState.yaw_rad += currentState.yaw_rate * dt;

    float v = compute_LQR_balancing_voltage(currentState, target, rad_offset);
    float dv = compute_yaw_voltage(currentState.yaw_rad, currentState.yaw_rate, yaw_ref);

    // u_L = v - dv,  u_R = v + dv
    shared_motor_voltage_L = v +   dv;
    shared_motor_voltage_R = v - dv;

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// --- 任务：Servo 控制 (Core 0) ---
void TaskServoCode(void *pvParameters)
{
  static int servo_angle = 60; // persist across iterations; start at a safe mid-range angle
  for (;;)
  {
    int target_angle = constrain(Servo_angle, 20, 100);
    int step = 5;
    if (abs(target_angle - servo_angle) <= step) {
      servo_angle = target_angle;
    } else {
      servo_angle += (target_angle > servo_angle) ? step : -step;
    }

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
  char buffer[600];
  for (;;)
  {
    snprintf(buffer, sizeof(buffer),
             "meter_error:%.2f x1:%.3f x2:%.3f x3:%.4f x4:%.3f "
             "cur_pos:%.3f tgt_pos:%.3f pos_sp:%.3f "
             "cur_vel:%.3f tgt_vel:%.3f vel_ff:%.3f "
             "cur_pitch:%.4f tgt_pitch:%.4f "
             "cur_gyro:%.3f tgt_gyro:%.3f "
             "Pitch_Rad:%.3f Voltage:%.2f VL:%.2f VR:%.2f "
             "yaw_rad:%.3f yaw_rate:%.3f yaw_e:%.3f yaw_mpu:%.3f "
             "Left_Velocity:%.2f Right_Velocity:%.2f Temp:%.1f\n",
             currentState.pitch_rad - rad_offset,
             x1, x2, x3, x4,
             currentState.position,  target.position,   pos_setpoint,
             currentState.velocity,  target.velocity, vel_ff_ramp,
             currentState.pitch_rad, rad_offset + target.pitch_rad,
             currentState.gyro_rate, target.gyro_rate,
             currentState.pitch_rad - rad_offset,
             (shared_motor_voltage_L + shared_motor_voltage_R) * 0.5f,
             shared_motor_voltage_L, shared_motor_voltage_R,
             currentState.yaw_rad, currentState.yaw_rate,
             yaw_e, yaw_mpu,
             motorL.shaftVelocity(), motorR.shaftVelocity(),
             mpu.getTemp());
    ws.cleanupClients();
    ws.textAll(buffer);
    // Serial.print(buffer);

    vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms更新一次网页监视器
  }
}

// void TaskcontrolCode(void *pv)
// {
//   //Task for reading serial input and updating control parameters
//   for (;;)
//   {
//     if (Serial.available()){
//       switch (Serial.read())
//       {
//       case '1':
//         applyProfile(0);
//         break;
//       case '2':
//         applyProfile(1);
//         break;
//       case '3':
//         applyProfile(2);
//         break;
//       case '4':
//         applyProfile(3);
//         break;
//       }

//     }
//   }
// }
void setup()
{
  Serial.begin(115200);
  // MPU6050
  // delay(2000); // wait for MPU6050 to power up before I2C init
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

  setupWebServer();

  // 创建任务 (注意优先级：电机最高)
  applyProfile(0);
  xTaskCreatePinnedToCore(TaskMotorCode, "MotorTask", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskBalanceCode, "BalanceTask", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMonitorCode, "MonitorTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskServoCode, "ServoTask", 5000, NULL, 1, NULL, 0);
  // xTaskCreatePinnedToCore(TaskcontrolCode, "ControlTask", 5000, NULL, 1, NULL, 0);

  
}
void loop() {}