#include "config.h"

volatile float Pitch_rad = 0.0, Pitch_gyro = 0.0;
volatile float shared_motor_voltage_L = 0.0;
volatile float shared_motor_voltage_R = 0.0;

float pitch_comp = 0.0;
RobotState currentState;
RobotState target = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 目标状态：位置0，速度0，倾斜角0，陀螺仪角速度0

MPU6050 mpu(Wire);
BLDCMotor motorL(7), motorR(7);
BLDCDriver3PWM driverL(14, 13, 12, 11), driverR(16, 15, 7, 6);
Encoder encoderL(3, 46, 1024), encoderR(17, 18, 1024);
MyServo LeftServo("LeftServo"), RightServo("RightServo");

int active_profile = 0;

void applyProfile(int idx)
{
  if (idx < 0 || idx >= N_PROFILES)
    return;
  active_profile = idx;
  const Profile &p = profile_list[idx];
  Servo_angle = p.servo_angle;
  rad_offset = p.rad_offset;
  K1 = p.K1;
  K2 = p.K2;
  K3 = p.K3;
  K4 = p.K4;
  K5 = p.K5;
  K6 = p.K6;
  // Snap trajectory to current position so robot stays in place after switch
  float cur_pos = get_average_distance_meters();
  pos_setpoint = cur_pos;
  target.position = cur_pos;
  position_offset = 0.0f;
  target.velocity = 0.0f;
  yaw_ref = currentState.yaw_rad;
  Serial.printf("[profile] switched to %d — servo=%d° K3=%.2f K4=%.2f offset=%.3f\n",
                idx, p.servo_angle, p.K3, p.K4, p.rad_offset);
}

void doLA() { encoderL.handleA(); }
void doLB() { encoderL.handleB(); }
void doRA() { encoderR.handleA(); }
void doRB() { encoderR.handleB(); }

// --- 任务：电机控制 (Core 1) ---
void TaskMotorCode(void *pv)
{
  delay(1000); 
  motorL.initFOC();
  motorR.initFOC();

  // Airborne detection (wheels lifted off the ground).
  // A simple velocity threshold of 40 rad/s was too low: at 0.8 m/s forward the
  // wheel-speed sum is already ~61 rad/s, so it false-triggered while driving.
  // When the robot is lifted, the (near-) unloaded wheels free-spin far above any
  // achievable ground speed, so we set the threshold well above ground cruise and
  // require the condition to persist (debounce) to reject noise.
  const float AIRBORNE_VEL_SUM = 90.0f; // rad/s, |velL|+|velR| (>> 0.8 m/s ground)
  const uint8_t AIRBORNE_DEBOUNCE = 20; // ticks @1 kHz ≈ 20 ms sustained
  uint8_t airborne_count = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    float pitch_err = fabs(currentState.pitch_rad - rad_offset);
    bool fallen = pitch_err > 0.4f;

    // Free-spin + near-upright => almost certainly lifted off the ground.
    bool airborne_now = ((fabs(motorL.shaftVelocity()) + fabs(motorR.shaftVelocity())) > AIRBORNE_VEL_SUM) && (pitch_err < 0.08f);
    if (airborne_now)
    {
      if (airborne_count < 255)
        airborne_count++;
    }
    else if (airborne_count > 0)
      airborne_count--;
    bool airborne = airborne_count >= AIRBORNE_DEBOUNCE;

    if (fallen || airborne)
    {
      if (motorL.enabled)
      {
        motorL.disable();
        motorR.disable();
      }
    }
    else
    {
      motorL.loopFOC();
      motorR.loopFOC();
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

void TaskBalanceCode(void *pv)
{
  const float dt = 0.005f; // 200 Hz fixed cadence (matches vTaskDelayUntil below)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    mpu.update();
    Pitch_rad = kalmanUpdate(mpu.getAngleY(), mpu.getGyroY(), dt);
    static float gyro_filtered = 0.0f;
    gyro_filtered = 0.95f * mpu.getGyroY() + 0.05f * gyro_filtered;
    Pitch_gyro = gyro_filtered;

    currentState.position = get_average_distance_meters();
    currentState.velocity = get_average_velocity_mps();
    currentState.pitch_rad = Pitch_rad * (PI / 180.0f);
    currentState.gyro_rate = Pitch_gyro * (PI / 180.0f);

    // yaw rate from wheel speed differential; positive => turning left (CCW from above)
    currentState.yaw_rate = constrain((motorR.shaftVelocity() - motorL.shaftVelocity()) * WHEEL_RADIUS_M / WHEEL_BASE_M, -6.28f, 6.28f);
    currentState.yaw_rad += currentState.yaw_rate * dt;

    float v = compute_LQR_balancing_voltage(currentState, target, rad_offset);
    float dv = compute_yaw_voltage(currentState.yaw_rad, currentState.yaw_rate, yaw_ref);

    // u_L = v + dv,  u_R = v - dv   (sign verified on this robot — turns correct way)
    shared_motor_voltage_L = v + dv;
    shared_motor_voltage_R = v - dv;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
  }
}

void TaskServoCode(void *pvParameters)
{
  const float SERVO_SPEED_DPS = 20.0f;
  for (;;)
  {
    int target_angle = constrain(Servo_angle, 20, 100);

    int right_trim  = (int)roundf(0.08f * target_angle);
    int right_angle = constrain(target_angle + right_trim, 20, 100);

    LeftServo.speedControl(180 - target_angle, SERVO_SPEED_DPS);
    RightServo.speedControl(right_angle,      SERVO_SPEED_DPS);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --- 任务：监控 (Core 0) ---
void TaskMonitorCode(void *pv)
{
  char buffer[768];
  for (;;)
  {
    snprintf(buffer, sizeof(buffer),
             "x1:%.3f x2:%.3f x3:%.4f x4:%.3f "
             "cur_pos:%.3f tgt_pos:%.3f pos_sp:%.3f "
             "cur_vel:%.3f tgt_vel:%.3f vel_ff:%.3f "
             "cur_pitch:%.4f tgt_pitch:%.4f "
             "cur_gyro:%.3f tgt_gyro:%.3f "
             "Voltage:%.2f VL:%.2f VR:%.2f "
             "yaw_rad:%.3f yaw_rate:%.3f yaw_ref:%.3f yaw_e:%.3f yaw_mpu:%.3f "
             "K1:%.2f K2:%.2f K3:%.2f K4:%.2f K5:%.3f K6:%.3f offset:%.3f profile:%d "
             "Left_Velocity:%.2f Right_Velocity:%.2f Temp:%.1f\n",
             x1, x2, x3, x4,
             currentState.position, target.position, pos_setpoint,
             currentState.velocity, target.velocity, vel_ff_ramp,
             currentState.pitch_rad, rad_offset + target.pitch_rad,
             currentState.gyro_rate, target.gyro_rate,
             (shared_motor_voltage_L + shared_motor_voltage_R) * 0.5f,
             shared_motor_voltage_L, shared_motor_voltage_R,
             currentState.yaw_rad, currentState.yaw_rate, yaw_ref, yaw_e, yaw_mpu,
             K1, K2, K3, K4, K5, K6, rad_offset, active_profile,
             motorL.shaftVelocity(), motorR.shaftVelocity(),
             mpu.getTemp());

    // printf("Pitch complementary: %.4f\n", mpu.getAngleY()* (PI / 180.0f));
    ws.cleanupClients();
    ws.textAll(buffer);

    vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms更新一次网页监视器
  }
}

void setup()
{
  Serial.begin(115200);
  // MPU6050
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  mpu.begin();
  mpu.setGyroOffsets(-1.484977, -0.755755, -2.337404);
  mpu.setAccOffsets(-0.003804, -0.074246, 0.216630);

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

  // Servo — write initial position immediately after ledcAttach so the servo
  // never sees the 0µs idle signal that would send it to its mechanical stop.
  {
    int sa         = constrain(Servo_angle, 20, 100);
    int right_trim = (int)roundf(0.06f * sa - 1.3f);
    LeftServo.setup(SERVO_L_PIN, 1000, 2000);
    LeftServo.write(180 - sa);
    RightServo.setup(SERVO_R_PIN, 1000, 2000);
    RightServo.write(constrain(sa + right_trim, 20, 100));
  }

  setupWebServer();

  // 创建任务 (注意优先级：电机最高)
  applyProfile(0);
  xTaskCreatePinnedToCore(TaskMotorCode, "MotorTask", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskBalanceCode, "BalanceTask", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMonitorCode, "MonitorTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskServoCode, "ServoTask", 5000, NULL, 1, NULL, 0);
#ifdef USE_XBOX_CONTROLLER
  xTaskCreatePinnedToCore(TaskControllerCode, "CtrlTask", 8000, NULL, 1, NULL, 0);
#endif
}

void loop() {}
