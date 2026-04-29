#include <Arduino.h>
#include "MyServo.h"

// put function declarations here:
void Read_Angle();

// Sevo Declay
MyServo Servo_1("Servo_1");

// Variable Declay
int angle = 0;

void setup()
{
  // put your setup code here, to run once:
  Servo_1.setup(GPIO_NUM_10, 500, 2500);
  gpio_set_direction(GPIO_NUM_11, GPIO_MODE_INPUT);
}

void loop()
{
  // put your main code here, to run repeatedly:
for (angle = 0; angle <= 180; angle++) {
    Servo_1.write(angle);
    Read_Angle();
    delay(100); // Give the servo time to move 1 degree
  }

  // Sweep back from 180 to 0
  for (angle = 180; angle >= 0; angle--) {
    Servo_1.write(angle);
    Read_Angle();
    delay(100);
  }
}

// put function definitions here:
void Read_Angle()
{
  float voltage = analogRead(GPIO_NUM_11);
  float Actual_Voltage = ((voltage / 4095.0) * 3.3)*1.5;
  float angle_read = Actual_Voltage/5*180;
  printf("angle: %.2f\n", angle_read);
}