#include <Arduino.h>
#include "MyServo.h"

// put function declarations here:
MyServo legServo("Leg_Servo");

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  legServo.setup(GPIO_NUM_10, 600, 2400);
}
void loop()
{
  // put your main code here, to run repeatedly:
  printf("Angle_Control_Start\n");
  legServo.write(0); // set servo angle
  delay(000);
  legServo.write(90); // set servo angle
  delay(1000);
  printf("Speed_Control_Start\n");
  legServo.speedControl(180,1);
  legServo.debug();
  delay(1800);
  legServo.speedControl(0,0.5);
  legServo.debug();
  delay(3600);
}
