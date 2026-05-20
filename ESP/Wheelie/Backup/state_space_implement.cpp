#include "config.h"

// --- 变量定义 ---
float angle_offset = -9.2; // for servo angle =25
volatile float Pitch_angle = 0.0, Pitch_gyro = 0.0;
float Pitch_integral = 0.0;

volatile float shared_motor_voltage = 0.0; // 共享变量，供平衡控制任务和电机控制任务使用
float output_voltage = 0.0;
float move_velocity = 0; // in voltage
int Servo_angle = 25;

// 1. 定义状态结构体
struct RobotState
{
  float position;
  float velocity;
  float pitch_angle;
  float gyro_rate;
};

// 2. 将之前计算出的 K 增益向量作为常量放入代码
const float K1 = 12.2474 / 15; // 轮子水平位置反馈 (Position)
const float K2 = 275.986 / 15; // 轮子水平速度反馈 (Velocity)
const float K3 = 91.2049 / 1;  // 车身倾斜角度反馈 (Pitch Angle in Rad)
const float K4 = 10.4469 / 4;  // 车身陀螺仪角速度反馈 (Gyro Rate in Rad/s)
float x1 = 0;
float x2 = 0;
float x3 = 0;
float x4 = 0;
// 3. 计算平衡电压的函数
float compute_LQR_balancing_voltage(RobotState current, RobotState target, float angle_offset_deg)
{
  // 转换为弧度
  float angle_offset_rad = angle_offset_deg * (PI / 180.0f);

  // 计算误差向量
  x1 = current.position - target.position;
  x2 = current.velocity - target.velocity;
  x3 = current.pitch_angle - angle_offset_rad - target.pitch_angle;
  x4 = current.gyro_rate - target.gyro_rate;

  // 状态空间核心：u = -(K*x)
  // 注意：这里手动加上负号，实现负反馈
  float voltage = (K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);
  // printf("LQR Voltage: %f | x1: %f, x2: %f, x3: %f, x4: %f\n", voltage, x1, x2, x3, x4);
  return constrain(voltage, -6.0f, 6.0f);
}

// 4. 辅助函数（用于获取米和米/秒）
float get_average_distance_meters()

{
  // 假设轮半径 r = 0.026m
  return (encoderL.getAngle() + encoderR.getAngle()) / 2.0f * 0.026f;
}

float get_average_velocity_mps()
{
  return (motorL.shaft_velocity + motorR.shaft_velocity) / 2.0f * 0.026f;
}

float YawKp = 0.05, YawKi = 0.0, YawKd = 0.01; // for servo angle =25
float turn_velocity = 0;                       // in voltage
volatile float Yaw_angle = 0.0, Pre_yaw_angle = 0.0, Yaw_gyro = 0.0;

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

// void TaskAutoCalibrate(void *pvParameters)
// {
//   for (;;)
//   {
//     if (is_calibrating)
//     {
//       if (abs(Pitch_angle) < 10.0)
//       {
//         // 1. Get the current motor speed (using SimpleFOC's velocity or your PID output)
//         // We use a simple Low Pass Filter to smooth out the noisy speed data
//         float current_speed = (motorL.shaft_velocity + motorR.shaft_velocity) / 2.0;
//         printf("current_speed: %f\n", current_speed);
//         average_speed = (average_speed * 0.95) + (current_speed * 0.05);

//         // 2. Adjust the offset based on the speed drift
//         if (average_speed > 0.1)
//         {
//           // Drifting forward: Lean it backward slightly
//           angle_offset += calibration_step;
//         }
//         else if (average_speed < -0.1)
//         {
//           // Drifting backward: Lean it forward slightly
//           angle_offset -= calibration_step;
//         }
//         else if (abs(average_speed) < 0.05)
//         {
//           // Speed is practically zero! We found the sweet spot.
//           printf("Calibration Complete! New Angle Offset: %f\n", angle_offset);
//           is_calibrating = false; // Stop calibrating
//         }
//       }
//     }
//     vTaskDelay(100 / portTICK_PERIOD_MS); // Run every 100ms
//   }
// }
// --- 任务：电机控制 (Core 1) ---
void TaskMotorCode(void *pv)
{
  delay(2000);
  motorL.initFOC();
  motorR.initFOC();
  // printf("Motor L Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorL.zero_electric_angle, motorL.sensor_direction);
  // printf("Motor R Zero Electric Angle: %.2f, Sensor Direction: %d\n", motorR.zero_electric_angle, motorR.sensor_direction);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // 电机环路必须高速运行
    motorL.loopFOC();
    motorR.loopFOC();
    if (abs(Pitch_angle) < 0.3)
    {
      motorL.move(shared_motor_voltage); // 注意：这里加负号实现负反馈
      motorR.move(shared_motor_voltage); // 注意：这里加负号实现负反馈
    }
    else
    {
      motorL.move(0.0f); // 注意：这里加负号实现负反馈
      motorR.move(0.0f); // 注意：这里加负号实现负反馈
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

// --- 任务：姿态计算 (Core 0) ---
void TaskBalanceCode(void *pv)
{
  // const TickType_t xFrequency = 1; // 1kHz
  // TickType_t xLastWakeTime = xTaskGetTickCount();
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
    currentState.position = get_average_distance_meters() - 0.2f;
    currentState.velocity = get_average_velocity_mps();
    currentState.pitch_angle = Pitch_angle * (PI / 180.0f); // 传感器转弧度
    currentState.gyro_rate = Pitch_gyro * (PI / 180.0f);    // 传感器转弧度/秒

    RobotState target = {0.0f, 0.0f, 0.0f, 0.0f};
    Pitch_angle = currentState.pitch_angle;
    // 使用你新定义的函数计算电压
    shared_motor_voltage = compute_LQR_balancing_voltage(currentState, target, angle_offset);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    // vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
    // Serial.printf("Motor_Speed: %.2f | Voltage:%.2f | Pitch Angle: %.2f\n",
    //               motorL.shaftVelocity(), shared_motor_voltage, Pitch_angle);
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

  // Servo
  LeftServo.setup(SERVO_L_PIN, 1000, 2000);
  RightServo.setup(SERVO_R_PIN, 1000, 2000);

  // 创建任务 (注意优先级：电机最高)
  xTaskCreatePinnedToCore(TaskMotorCode, "MotorTask", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskBalanceCode, "BalanceTask", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMonitorCode, "MonitorTask", 5000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskServoCode, "ServoTask", 5000, NULL, 1, NULL, 0);
  // xTaskCreatePinnedToCore(TaskAutoCalibrate, "AutoCalTask", 5000, NULL, 1, NULL, 0);
}
void loop() {}