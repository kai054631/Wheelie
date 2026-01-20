#include <MPU6050_light.h>
MPU6050 mpu(Wire);

#include <SimpleFOC.h>
Encoder sensor = Encoder(2, 3, 1024);
void doA() {
  sensor.handleA();
}
void doB() {
  sensor.handleB();
}
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 11, 9, 8);
BLDCMotor motor = BLDCMotor(7, 3.2, 220);
int setPoint = 0;
float Kp = 0.1;
void setMotor(float speed) {
  motor.loopFOC();
  motor.move(speed);
}
float getAngle() {
  mpu.update();
  float angle = mpu.getAngleY();
  return angle;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  byte status = mpu.begin();

  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");

  //motor encoder initialise
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  sensor.init();
  
  sensor.enableInterrupts(doA, doB);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  //driver.init();
  motor.linkDriver(&driver);
  motor.PID_velocity.P = 0.05;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  motor.voltage_sensor_align = 3;  // aligning voltage [V]
  motor.voltage_limit = 1;
  motor.controller = MotionControlType::velocity;  //(velocity,torque,angle)
  motor.velocity_limit = 4;
  ;
  motor.init();
  //motor.initFOC();
}

void loop() {
  // MUST run as fast as possible
  motor.loopFOC();

  static unsigned long last_time = 0;
  if (millis() - last_time >= 10) {
    last_time = millis();
    
    // 1. UPDATE FIRST
    mpu.update();
    float currentAngle = mpu.getAngleY();

    // 2. THE NAN TRAP: If sensor fails, stop motor and skip this loop
    if (isnan(currentAngle)) {
      motor.move(0); 
      // Optional: Serial.println("I2C Error!"); 
      return; 
    }
    
    // 3. Calculate and move
    float error = currentAngle - setPoint;
    float output = error * Kp;
    motor.move(output);

    static int print_decimator = 0;
    if (print_decimator++ >= 25) {
      Serial.print(error);
      Serial.print(" ");
      Serial.println(output);
      print_decimator = 0;
    }
  }
}
