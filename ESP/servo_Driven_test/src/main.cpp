#include <Arduino.h>
#include "driver/gpio.h"

const int min_us=500;
const int max_us=2500;
const int bit=10;
//float min_angle = (min_us / 20000) * (pow(2, bit));
int min_angle = 26;
int max_angle = 128;
//float max_angle = (max_us / 20000) * (pow(2, bit));

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  ledcSetup(0, 50, bit);         // CHANNEL,HZ,ADC BIT
  ledcAttachPin(GPIO_NUM_5, 0); // PIN NUMBER,CHANNEL8)
  ledcSetup(1, 50, bit); 
  ledcAttachPin(GPIO_NUM_6, 1); // PIN NUMBER,CHANNEL8)
  printf("Servo Initialized");
}

void loop()
{
  // put your main code here, to run repeatedly:
  for (int i = min_angle; i < max_angle; i++)
  {
    printf("%i\n",i);
    ledcWrite(0, i); //servo Write
    ledcWrite(1, i);
    delay(50);
  }
  for (int i = max_angle; i > min_angle; i--)
  {
    printf("%i\n",i);
    ledcWrite(0, i);
    ledcWrite(1, i);
    delay(50);
  }
}