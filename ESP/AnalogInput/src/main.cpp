/*
set pin mode
analogread
digitalOUTPUT

*/
#include <Arduino.h>

// put function declarations here:
#define led_1 GPIO_NUM_4
#define led_2 GPIO_NUM_5
#define LDR GPIO_NUM_18
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gpio_set_direction(led_1,GPIO_MODE_OUTPUT);
  gpio_set_direction(led_2,GPIO_MODE_OUTPUT);
  gpio_set_direction(LDR,GPIO_MODE_INPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  printf("%d\n",analogRead(LDR));
  if(analogRead(LDR)>300){
    gpio_set_level(led_1,1);
    gpio_set_level(led_2,0);
  }else{
    gpio_set_level(led_1,0);
    gpio_set_level(led_2,1);
  }

}

