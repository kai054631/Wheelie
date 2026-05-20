#include "config.h"

// --- 变量定义 ---
float angle_offset = -9.2; // in angle form
volatile float Pitch_Radian = 0.0, Pitch_gyro = 0.0;

volatile float shared_motor_voltage = 0.0; // share variable for taskbalance and taskmotor
int Servo_angle = 25;                      // angle for the servo

bool runMotor = false;
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
  if (!runMotor)
  {
    vTaskDelete(NULL); // 如果不运行电机，直接删除任务
    return;
  }
  else
  {
    printf("Starting Motor Initialization and FOC Alignment...\n");
    motorL.initFOC();
    motorR.initFOC();
    // printf("Motor L Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorL.zero_electric_angle, motorL.sensor_direction);
    // printf("Motor R Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorR.zero_electric_angle, motorR.sensor_direction);
  }
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    motorL.loopFOC();
    motorR.loopFOC();
    if (abs(Pitch_Radian) < 0.3)
    {
      motorL.enable();
      motorR.enable();
      motorL.move(shared_motor_voltage);
      motorR.move(shared_motor_voltage);
    }
    else
    {
      motorL.disable();
      motorR.disable();
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
    Pitch_Radian = kalmanUpdate(mpu.getAngleY(), mpu.getGyroY(), dt);
    Pitch_gyro = mpu.getGyroY();

    RobotState currentState;
    currentState.position = get_average_distance_meters();
    currentState.velocity = get_average_velocity_mps();
    currentState.pitch_angle = Pitch_Radian * (PI / 180.0f); // in radian/s
    currentState.gyro_rate = Pitch_gyro * (PI / 180.0f);     // in rad/s

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
    if (Servo_angle <= 100)
    {
      int servo_angle = constrain(Servo_angle, 20, 100);
      LeftServo.write(180 - servo_angle);                
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
    // Serial.printf("Motor_Speed: %.2f | Voltage:%.2f | Pitch Angle: %.2f\n", motorL.shaftVelocity(), shared_motor_voltage, Pitch_angle);
    Serial.printf("X1: %.2f | X2: %.2f | X3: %.2f | X4: %.2f\n", x1, x2, x3, x4);
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

  // setupWebServer();
  // vTaskDelay(2000 / portTICK_PERIOD_MS);

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
  runMotor = true;

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