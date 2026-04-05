/*
digitalWrite led on
Serial monitor
AnalogWrite PWM

*/

#include <Arduino.h>
#include "driver/gpio.h"

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);

    // arduino pin config
    //pinMode(4, OUTPUT); // Set GPIO 4 as an output

    // esp pwm pin config
    ledcSetup(0, 25000, 10);
    ledcAttachPin(GPIO_NUM_5, 0);

    // stop code run without serial
    while (!Serial){

    }
    printf("Setup complete!\n");
}

void loop()
{
    // Main code here - runs repeatedly

    // esp pin config 1 (single pin)
    gpio_pad_select_gpio(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4,GPIO_MODE_OUTPUT);//pinMode
    // serial print in esp
    printf("LED on\n");
    gpio_set_level(GPIO_NUM_4,1); // Turn on the LED connected to GPIO 4
    delay(1000);           // Wait for 1 second

    printf("LED off\n");
    
    gpio_set_level(GPIO_NUM_4,0); // digitalWrite(pin)
    delay(1000);                   // Wait for 1 second

    printf("LED PWM on\n");
    for (int i = 0; i < 1024; i++)
    {
        printf("%i\n",i);
        ledcWrite(0, i); // analogWrite in esp
        delay(10);
    }
    delay(1000);
}