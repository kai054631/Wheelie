/*description
author :Yong Sheng Kai
Last modify date: 25/1/2026
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
float compAngleY = 0;  // The filtered result
unsigned long lastFilterTime = 0;
const float alpha = 0.99;  // Filter coefficient
//Encoder Setup
Encoder sensor = Encoder(2, 3, 1024);
unsigned long lastMillis = 0;
const long interval = 200;
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
void applyComplementaryFilter() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastFilterTime) / 1000000.0;  // Convert to seconds
  lastFilterTime = currentTime;

  // 1. Get raw data from MPU6050_light
  mpu.update();
  float gyroRateY = mpu.getGyroY();        // Degrees per second
  float accelAngleY = mpu.getAccAngleY();  // Angle based on gravity

  // 2. The Complementary Filter Math
  compAngleY = alpha * (compAngleY + gyroRateY * dt) + (1.0 - alpha) * accelAngleY;
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
  motor.voltage_limit = 4.0;
  motor.controller = MotionControlType::velocity;  //(velocity,torque,angle)
  motor.velocity_limit = 10;

  //motor.useMonitoring(Serial);
  //SimpleFOCDebug::enable(&Serial);
  motor.init();
  motor.initFOC();

  Serial.println(F("Motor ready."));
  delay(1000);
  digitalWrite(13, HIGH);
}

void loop() {
  motor.loopFOC(); // Keep FOC running
  mpu.update();
  if(Serial.available()>0){
    int val=Serial.parseInt();
    setServoAngle(val);
  }
  float currentAngle = mpu.getAngleY();
  
  // Simple logic to see if values change
  target_velocity = (abs(currentAngle) > 1.0) ? (currentAngle * -1.0) : 0;
  target_velocity = constrain(target_velocity, -20, 20);
  
  motor.move(target_velocity);

  unsigned long currentmillis = millis();
  if (currentmillis - lastMillis >= interval) {
    lastMillis = currentmillis;

    // We use simple labels to avoid buffer overflow
    Serial.print("A:"); Serial.print(currentAngle);
    Serial.print("\tT:"); Serial.print(target_velocity);
    Serial.print("\tV:"); Serial.println(motor.shaft_velocity);

    // Flash LED
    digitalWrite(13, !digitalRead(13));
  }
}