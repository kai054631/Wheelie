/*
RTOS

*/
#include <Arduino.h>
#include "driver/gpio.h"
// put function declarations here:
#define led_1 GPIO_NUM_4
#define led_2 GPIO_NUM_5
#define ldr GPIO_NUM_18
#define button GPIO_NUM_21

TaskHandle_t BlinkTaskHandle = NULL;

// volatile bool taskSuspend = false;
// volatile uint32_t lastInterruptTime = 0;
// const uint32_t debounceDelay = 200; // debounce period
// volatile bool button_press = false;

// void IRAM_ATTR buttonISR()
// {
//   // Debounce
//   uint32_t currentTime = millis();
//   if (currentTime - lastInterruptTime < debounceDelay)
//   {
//     return;
//   }
//   lastInterruptTime = currentTime;

//   // Toggle task state
//   button_press = !button_press;
//   if (button_press)
//   {
//     vTaskSuspend(BlinkTaskHandle);
//     Serial.println("BlinkTask Suspended");
//   }
//   else
//   {
//     vTaskResume(BlinkTaskHandle);
//     Serial.println("BlinkTask Resumed");
//   }
// }
void blink(void *parameter)
{
  for (;;)
  {
    gpio_set_level(led_1, 1);
    Serial.printf("LED On\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(led_1, 0);
    Serial.printf("LED Off\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
void ReadSensor(void *parameter)
{
  for (;;)
  {
    int light = analogRead(ldr);
  }
}
void setup()
{
  // put your setup code here, to run once:
  gpio_set_direction(led_1, GPIO_MODE_OUTPUT);
  gpio_set_direction(led_2, GPIO_MODE_OUTPUT);
  gpio_set_direction(ldr, GPIO_MODE_INPUT);
  gpio_set_direction(button, GPIO_MODE_INPUT);
  gpio_set_pull_mode(button, GPIO_PULLUP_ONLY);

  //attachInterrupt(digitalPinToInterrupt(button), buttonISR, FALLING);
  Serial.begin(115200);
  while (!Serial && millis() < 5000)
  {
    delay(10);
  }
  Serial.printf("Serial ready");

  xTaskCreatePinnedToCore(
      blink,
      "Turn_LED_on",
      3000,
      NULL,
      1,
      &BlinkTaskHandle,
      1);
  xTaskCreatePinnedToCore(
      ReadSensor,
      "read sensor",
      3000,
      NULL,
      1,
      &BlinkTaskHandle,
      1);
}

void loop()
{

  // put your main code here, to run repeatedly:
}
