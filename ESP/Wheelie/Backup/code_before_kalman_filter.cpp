// 2/5/2026
// main of wheelie before kalman filter

#include <Arduino.h>
#include "MyServo.h"
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050_light.h>
// put function declarations here:
//--------------------------------
// MPU6050 Declare
#define I2C_SDA 2
#define I2C_SCL 1

MPU6050 mpu(Wire);
//--------------------------------

//--------------------------------
// SimpleFOC Declare

// Driver_0 Left
BLDCMotor motorL = BLDCMotor(7);
BLDCDriver3PWM driverL = BLDCDriver3PWM(14, 13, 12, 11);

Encoder encoderL = Encoder(3, 46, 1024);

void doLA() { encoderL.handleA(); }
void doLB() { encoderL.handleB(); }

//--------------------------------
// Driver_1  Right
BLDCMotor motorR = BLDCMotor(7);
BLDCDriver3PWM driverR = BLDCDriver3PWM(16, 15, 7, 6);

Encoder encoderR = Encoder(17, 18, 1024);
void doRA() { encoderR.handleA(); }
void doRB() { encoderR.handleB(); }
//--------------------------------

//--------------------------------
// Servo Declare
// Servo_0 Left
MyServo LeftServo("LeftServo");
//--------------------------------

// Servo_1 Right
MyServo RightServo("RightServo");

//--------------------------------
// esp declare
TaskHandle_t TaskMotor;
TaskHandle_t TaskMonitor;
TaskHandle_t TaskBalance;
// 用于核间通信的变量
// R Motor
volatile float current_velocity = 0.0;
volatile float current_angle = 0.0;
float Kp = 0.18, Ki = 0.0, Kd = 0.04;

// Pitch angle for robot
volatile float Pitch_angle = 0.0;
volatile float Pitch_gyro = 0.0;
float output_voltage = 0.0;
float angle_offset = -82.0;
// task declare
void TaskMotorCode(void *pvParameters);   // code to run the foc algorithm
void TaskMonitorCode(void *pvParameters); // code to serial print per 100ms
void TaskBalanceCode(void *pvParameters); // code to read from mpu6050
//--------------------------------

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  // MPU6050 Setup
  //--------------------------------
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  while (!Serial)
    delay(10);
  Serial.println("Booting...");

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(100); // Give the bus time to settle

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  if (status != 0)
  {
    Serial.println(F("CRITICAL ERROR: MPU6050 not found. Stopping."));
    while (1)
    {
      delay(1000); // Sit here so it doesn't crash the rest of the app
    }
  }
  // mpu.calcOffsets();
  // Serial.println(F("校准完成！"));
  //--------------------------------

  // // SimpleFOC Setup
  // --- 左电机 (Motor L) 初始化 ---
  //--------------------------------
  encoderL.init(); // 若启用请解开注释
  encoderL.enableInterrupts(doLA, doLB);
  motorL.linkSensor(&encoderL);

  driverL.voltage_power_supply = 12;
  driverL.init();
  motorL.linkDriver(&driverL);

  motorL.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorL.controller = MotionControlType::torque;
  motorL.torque_controller = TorqueControlType::voltage;
  motorL.voltage_limit = 8;

  motorL.init();
  motorL.initFOC();
  //--------------------------------
  // --- 右电机 (Motor R) 初始化 ---
  encoderR.init();
  encoderR.enableInterrupts(doRA, doRB);
  motorR.linkSensor(&encoderR);

  driverR.voltage_power_supply = 12;
  driverR.init();
  motorR.linkDriver(&driverR);

  motorR.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorR.controller = MotionControlType::torque;
  motorR.torque_controller = TorqueControlType::voltage;
  motorR.voltage_limit = 8; // 如果电机发热，可调低此值

  motorR.init();
  motorR.initFOC();

  // --- 调试接口 ---
  // command.add('L', doMotorL, "Left_motor");
  // command.add('R', doMotorR, "Right_motor");

  //--------------------------------

  // Servo Setup
  //--------------------------------
  LeftServo.setup(GPIO_NUM_4, 1000, 2400);
  RightServo.setup(GPIO_NUM_5, 1000, 2000);

  // LeftServo.write(20);
  // RightServo.write(20);
  //--------------------------------

  // bt serial setup
  //  command.add('P', doPID, "PID balance");

  // 创建任务：电机控制任务 (Core 1)
  xTaskCreatePinnedToCore(
      TaskMotorCode, /* 任务函数 */
      "MotorTask",   /* 任务名称 */
      10000,         /* 堆栈大小 */
      NULL,          /* 任务参数 */
      1,             /* 优先级 */
      &TaskMotor,    /* 任务句柄 */
      1              /* 绑定到 Core 1 */
  );

  // 创建任务：监控打印任务 (Core 0)
  xTaskCreatePinnedToCore(
      TaskMonitorCode,
      "MonitorTask",
      10000,
      NULL,
      0, /* 低优先级 */
      &TaskMonitor,
      0 /* 绑定到 Core 0 */
  );

  xTaskCreatePinnedToCore(
      TaskBalanceCode,
      "BalanceTask",
      10000,
      NULL,
      1, /* 低优先级 */
      &TaskBalance,
      0 /* 绑定到 Core 0 */
  );
  delay(1000);
}

void loop()
{
}

// put function definitions here:
void TaskMotorCode(void *pvParameters)
{
  for (;;)
  {
    // printf("core foc working\n");

    float current_pitch = Pitch_angle;
    float target_pitch_angle = 0;
    float error = target_pitch_angle - current_pitch;

    if (current_pitch > 30.0f || current_pitch < -30.0f)
    {
      // Safety stop: pitch exceeded safe threshold
      error = 0;
      output_voltage = 0.0;
      motorL.move(0);
      motorR.move(0);
    }
    else
    {
      if ((error > 1) || (error < -1))
      {
        output_voltage = (Kp * error) - (Kd * Pitch_gyro);
        output_voltage = constrain(output_voltage, -8.0, 8.0);
        motorL.loopFOC();
        motorL.move(-output_voltage);
        motorR.loopFOC();
        motorR.move(-output_voltage); // 在这里执行你的 PID 平衡逻辑
      }
      else
      {
        error = 0;
        output_voltage = 0.0;
        motorL.move(0);
        motorR.move(0);
      }
    }

    // 更新全局变量以便 Core 0 读取
    current_velocity = motorR.shaftVelocity();
    current_angle = motorR.shaftAngle();

    vTaskDelay(1 / portTICK_PERIOD_MS); // 微小延时，让出 CPU
  }
}

void TaskMonitorCode(void *pvParameters)
{
  static bool headerPrinted = false;
  for (;;)
  {
    if (!headerPrinted)
    {
      printf("Kp,raw_angle,Pitch_angle,gyro,error,derivative,out_voltage\n");
      headerPrinted = true;
    }

    float error = -Pitch_angle; // setpoint is 0
    float derivative = Kd * Pitch_gyro;
    printf("%f,%f,%f,%f,%f,%f\n", Kp, mpu.getAngleY(), Pitch_angle, error, derivative, output_voltage);
    vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 打印一次
  }
}

void TaskBalanceCode(void *pvParameters)
{
  for (;;)
  {
    mpu.update();
    Pitch_angle = mpu.getAngleY() - angle_offset;

    // 2. 应用低通滤波 (Alpha = 0.2，平滑掉传感器的噪声)
    // Pitch_angle = (旧值 * 0.8) + (新读数 * 0.2)
    // Pitch_angle = (Pitch_angle * 0.8) + (raw_angle * 0.2);

    // 3. 陀螺仪数据通常也需要平滑，建议同样加入滤波
    float Pitch_gyro = mpu.getGyroY();
    // /Pitch_angle = (Pitch_angle * 0.8) + (raw_angle * 0.2);
    vTaskDelay(10 / portTICK_PERIOD_MS); // 20ms 打印一次
  }
}