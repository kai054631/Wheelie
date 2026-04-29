#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

// Explicitly define pins to avoid conflicts with your SimpleFOC driver
#define I2C_SDA 9
#define I2C_SCL 8

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for Serial Monitor on your T470

  // Initialize I2C with specific S3 pins
  Wire.begin(I2C_SDA, I2C_SCL);
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  
  // If status is not 0, it means the MPU is not found
  if(status != 0){ 
    Serial.println(F("Check your wiring! MPU6050 not found."));
    while(1); 
  }

  Serial.println(F("Calculating offsets, keep the robot still..."));
  delay(1000);
  mpu.calcOffsets(); // Zeroes the gyro and accelerometer
  Serial.println(F("Done!\n"));
}

void loop() {
  // MUST call update() as fast as possible
  mpu.update();

  // Print every 100ms so the Serial Monitor is readable
  if ((millis() - timer) > 100) { 
    Serial.print("X: ");
    Serial.print(mpu.getAngleX());
    Serial.print("\tY (Balance): ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");
    Serial.println(mpu.getAngleZ());
    timer = millis();
  }
}