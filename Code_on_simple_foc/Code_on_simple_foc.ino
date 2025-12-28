#include <SimpleFOC.h>

Encoder sensor = Encoder(2, 3, 1024);

void doA(){sensor.handleA();}
void doB(){sensor.handleB();}

BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);
BLDCMotor motor = BLDCMotor(7,3.2,220);

Commander command = Commander(Serial);
//for serial control
float target_val=0;
void doTarget(char* cmd) //Read command from serial
{ 
  command.scalar(&target_val, cmd); 
}
void setup() {
  // put your setup code here, to run once:
  sensor.init();
  sensor.enableInterrupts(doA, doB);
  motor.linkSensor(&sensor);
  
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.PID_velocity.P = 0.05;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  motor.voltage_sensor_align = 3;// aligning voltage [V]
  motor.voltage_limit = 5;
  motor.controller = MotionControlType::velocity;//(velocity,torque,angle)
  motor.velocity_limit = 4;

  
  
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  SimpleFOCDebug::enable(&Serial);
  motor.init();
  motor.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.loopFOC();
  motor.move();
    
    //command.run();
}
