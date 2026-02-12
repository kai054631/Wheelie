/*description
author :Yong Sheng Kai
Last modify date: 21/1/2026
Arduino UNO
MT6701 encoder
MG995 Servo
MPU6050 
//FYP 1 DEMO CODE

*/

#include <MPU6050_light.h>
#include <SimpleFOC.h>
#include <Servo.h>

MPU6050 mpu(Wire);
//Encoder Setup
Encoder sensor = Encoder(2, 3, 1024);
unsigned long lastMillis = 0;
const long interval = 100;
int count = 0;         //ppr count
float RPS = 0.0;       //wheel speed
bool ledstate = true;  // led on board to let user know if the program is freeze or not
//pid calculate
float error = 0.0, errorsum = 0.0, fpidout = 0.0;
float Kp = 1.0,
      Ki = 0.0,
      Kd = 0.0;
float fsetval = 0.0;  //set angle for motor
float target_velocity = 0.0;
//BLDC setup
BLDCMotor motor = BLDCMotor(7, 3.2, 220);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
void doA() {
  sensor.handleA();
}
void doB() {
  sensor.handleB();
}

//servo setup
#define ServoPin 6
Servo servoL;
void setServoAngle(int state) {
  //int us=map(angle,0,180,600,2400);
  switch (state) {
    case 0:
      servoL.attach(ServoPin);
      //servoL.write(60);
      servoL.writeMicroseconds(1600);  //stand up
      break;
    case 1:
      servoL.attach(ServoPin);
      //servoL.write(100);
      servoL.writeMicroseconds(1200);  //squad down 120degree
      break;
    default:
      break;
  }
}


int pidformotor() {                   //give motor data to run(probally need to return velocity/torque data)
  error = fsetval - mpu.getAngleY();  //calculate error from angle set
  errorsum = errorsum + error;
  fpidout = (Kp * error) + (Ki * errorsum);  //PI Implement
  if (fpidout >= 0) {
    if (fpidout > 255) {  //ensure fpidout does not more then 255
      fpidout = 255;
    } else {
      fpidout = fpidout;
    }
  } else if (fpidout < 0) {
    if (fpidout < -255) {  //ensure fpidout does not less then -255
      fpidout = -255;
    } else {
      fpidout = fpidout;
    }
  } else {  //set fpidout =0 if any issue occur
    fpidout = 0;
  }
  return (fpidout);
}
void setup() {
  // put your setup code here, to run once:
  //encoder setup
  pinMode(13, OUTPUT);  //led onboard
  Serial.begin(115200);
  //servo
  //servoL.attach(ServoPin);

  //mpu6050 setup
  Wire.begin();
  byte status = mpu.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcAccOffsets();

  //BLDC
  sensor.init();
  sensor.enableInterrupts(doA, doB);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.PID_velocity.P = 0.02;
  motor.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0.0;
  motor.LPF_velocity.Tf = 0.05;
  motor.voltage_sensor_align = 5;  // aligning voltage [V]
  motor.voltage_limit = 5;
  motor.controller = MotionControlType::velocity;  //(velocity,torque,angle)
  motor.velocity_limit = 10;

  motor.useMonitoring(Serial);
  SimpleFOCDebug::enable(&Serial);
  motor.init();
  motor.initFOC();

  Serial.println(F("Motor ready."));
  delay(1000);
  digitalWrite(13, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.loopFOC();
  motor.move(target_velocity);
  if (Serial.available()) {
    char a = Serial.read();
    int val = Serial.parseInt();
    if (a == 'S' || a == 's') {  //control servo to let robot squad down or stand up
      setServoAngle(val);
    }
    if (a == 'M' || a == 'm') {  //control servo to let robot squad down or stand up
      target_velocity = val * 6.28;

      //Serial.print(target_velocity);
    }
  }
  unsigned long currentmillis = millis();
  if (currentmillis - lastMillis >= interval) {
    lastMillis = currentmillis;

    ledstate = !ledstate;        //Invert LED state
    digitalWrite(13, ledstate);  //led onboard
    Serial.print("RPS:");
    Serial.print(motor.shaft_velocity / 6.28);  //encoder count for motor
    Serial.print(",");
    Serial.print("angle:");
    Serial.println(mpu.getAngleY());

    // Serial.print(",");
    // Serial.print("Error:");
    // Serial.print(error);  //error between angle set and current angle of robot
    // Serial.print(",");
    // Serial.print("Total Error:");
    // Serial.print(errorsum);  //sum of error between angle set and current angle of robot
    // Serial.print(",");
    // Serial.print("Motor PWM:");
    // Serial.print(pidformotor());  //value to motor for balance back the robot
    //Serial.print(",");
  }
}
