#include <Arduino.h>
#include "SimpleFOC.h"

// put function declarations here:
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(16, 15, 7, 6);

Encoder encoder = Encoder(4, 5, 1024);

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

Commander command = Commander(Serial);
void doMotor(char* cmd) { 
    command.motor(&motor, cmd); 
}
void setup() {
  // put your setup code here, to run once:
  encoder.enableInterrupts(doA, doB);
  
  // Link motor to encoder
  motor.linkSensor(&encoder);

  // Driver config
  driver.voltage_power_supply = 12; // Update based on your battery/PSU
  driver.init();
  motor.linkDriver(&driver);

  // FOC modulation and control loop
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::velocity; // Change to angle for position control

  // PID parameters (Start low and tune)
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.0f;
  motor.voltage_limit = 4; // Safety limit (Volts)

  // Serial monitoring
  //Serial.begin(115200);
  motor.useMonitoring(Serial);

  // Initialize motor
  motor.init();
  
  // Align encoder and start FOC
  motor.initFOC();

  // Add commander for tuning
  command.add('M', doMotor, "motor");

  printf("Motor ready. Target velocity: 5 rad/s");
  _delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.loopFOC();

  // Motion control (set target here)
  motor.move(5); // 5 rad/s

  // Commander interface
  command.run();

}

// put function definitions here:

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

void TaskBalanceCode(void *pvParameters)
{
  for (;;)
  {
    float RobotAngle=mpu.getA
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