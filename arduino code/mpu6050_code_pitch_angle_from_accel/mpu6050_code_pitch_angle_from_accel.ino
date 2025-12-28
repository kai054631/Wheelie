#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
float pitchFromAccel = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  pitchFromAccel = atan(-a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2)));	
  Serial.print(0);
  Serial.print(",");
  Serial.println(pitchFromAccel);
  delay(200);

}
