#include <Arduino.h>
#include <SimpleFOC.h>
//--------------------------------

//--------------------------------
// SimpleFOC Declare

// // Driver_0 Left
// BLDCMotor motorL = BLDCMotor(7);
// BLDCDriver3PWM driverL = BLDCDriver3PWM(14, 13, 12, 11);

// Encoder encoderL = Encoder(3, 46, 1024);

// void doLA() { encoderL.handleA(); }
// void doLB() { encoderL.handleB(); }

//--------------------------------
// Driver_1  Right
BLDCMotor motorR = BLDCMotor(7);
BLDCDriver3PWM driverR = BLDCDriver3PWM(16, 15, 7, 6);

Encoder encoderR = Encoder(17, 18, 1024);
void doRA() { encoderR.handleA(); }
void doRB() { encoderR.handleB(); }

//--------------------------------

// esp declare
TaskHandle_t TaskMotor;
TaskHandle_t TaskMonitor;

// 用于核间通信的变量
volatile float current_velocity = 0;
volatile float current_angle = 0;

// task declare
void TaskMotorCode(void *pvParameters);   // code to run the foc algorithm
void TaskMonitorCode(void *pvParameters); // code to serial print per 100ms

void setup()
{
  // // // put your setup code here, to run once:

  // // SimpleFOC Setup
  // ==========================================
  // Setup 函数内的代码整理
  // ==========================================

  Serial.begin(115200);

  // --- 左电机 (Motor L) 初始化 ---
  // encoderL.init(); // 若启用请解开注释
  // encoderL.enableInterrupts(doLA, doLB);
  // motorL.linkSensor(&encoderL);

  // driverL.voltage_power_supply = 12;
  // driverL.init();
  // motorL.linkDriver(&driverL);

  // motorL.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motorL.controller = MotionControlType::torque;
  // motorL.torque_controller = TorqueControlType::voltage;
  // motorL.voltage_limit = 5;

  // motorL.init();
  // motorL.initFOC();

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
  motorR.voltage_limit = 5; // 如果电机发热，可调低此值

  motorR.init();
  motorR.initFOC();

  // --- 调试接口 ---
  // command.add('L', doMotorL, "Left_motor");
  // command.add('R', doMotorR, "Right_motor");

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
}

void loop()
{
  // 保持为空，所有逻辑都在双核任务中
}

void TaskMotorCode(void *pvParameters)
{
  for (;;)
  {
    // printf("core foc working\n");
    motorR.loopFOC();
    motorR.move(0.5); // 在这里执行你的 PID 平衡逻辑

    // 更新全局变量以便 Core 0 读取
    current_velocity = motorR.shaftVelocity();
    current_angle = motorR.shaftAngle();

    vTaskDelay(1 / portTICK_PERIOD_MS); // 微小延时，让出 CPU
  }
}

void TaskMonitorCode(void *pvParameters)
{
  for (;;)
  {
    // printf("core balancing working");
    printf("Vel: %.2f | Angle: %.2f\n", current_velocity, current_angle);
    vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 打印一次
  }
}