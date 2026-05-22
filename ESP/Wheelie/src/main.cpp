#include "config.h"

// --- 变量定义 ---
float angle_offset = -8.6; // in angle form
volatile float Pitch_angle = 0.0, Pitch_gyro = 0.0;
volatile float shared_motor_voltage = 0.0; // share variable for taskbalance and taskmotor
int Servo_angle = 25;                      // angle for the servo

bool runMotor = false;

float K1 = -0.0f; // 轮子水平位置反馈 (Position) --- IGNORE ---
float K2 = -0.0f; // 轮子水平速度反馈 (Velocity) --- IGNORE ---
float K3 = -5.0f; // 车身倾斜角度反馈 (Pitch Angle in Rad) --- IGNORE ---
float K4 = -0.0f; // 车身陀螺仪角速度
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
    if (abs(Pitch_angle) > 20.0f)
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
      motorL.move(shared_motor_voltage);
      motorR.move(shared_motor_voltage);
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
    Pitch_gyro = mpu.getGyroY();

    RobotState currentState;
    currentState.position = get_average_distance_meters();
    currentState.velocity = get_average_velocity_mps();
    currentState.pitch_angle = Pitch_angle * (PI / 180.0f); // in radian/s
    currentState.gyro_rate = Pitch_gyro * (PI / 180.0f);    // in rad/s
    RobotState target = {0.0f, 0.0f, 0.0f, 0.0f};           // 目标状态：位置0，速度0，倾斜角0，陀螺仪角速度0
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
    // Serial.printf(" x1: %.2f, x2: %.2f ,x3: %.2f ,x4: %.2f , Pitch_Angle: %.2f , Voltage: %.2f , Left_Velocity: %.2f , Right_Velocity: %.2f, Motor_L_Angle: %.2f, Motor_R_Angle: %.2f\n", x1, x2, x3, x4, Pitch_angle, shared_motor_voltage, motorL.shaftVelocity(), motorR.shaftVelocity(), encoderL.getAngle(), encoderR.getAngle());
    if (Serial.available() > 0)
    {
      String input = Serial.readStringUntil('\n');
      input.trim();        // Remove invisible \r or spaces at the end
      input.toLowerCase(); // Convert to lowercase so 'K1' or 'k1' both work

      int k_index;
      float new_value;

      // sscanf looks for "k", then an integer (%d), a space, and a float (%f)
      // It returns the number of successfully matched items (we want 2)
      if (sscanf(input.c_str(), "k%d %f", &k_index, &new_value) == 2)
      {

        // Assign the value based on the index parsed
        if (k_index == 1)
          K1 = new_value;
        else if (k_index == 2)
          K2 = new_value;
        else if (k_index == 3)
          K3 = new_value;
        else if (k_index == 4)
          K4 = new_value;
        else
        {
          Serial.println("Invalid K index! Use 1, 2, 3, or 4.");
          return;
        }

        // Print the updated values (using ESP32's printf support)
        Serial.printf("Updated K%d to %.2f\n", k_index, new_value);
        Serial.printf("Current Status -> K1: %.2f | K2: %.2f | K3: %.2f | K4: %.2f\n", K1, K2, K3, K4);
      }
      else
      {
        Serial.println("Unrecognized command. Try format: k1 10.5");
      }
    }

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
  // runMotor = true;

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