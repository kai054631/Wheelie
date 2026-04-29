#include <Arduino.h>

// put function declarations here:
int EncoderLA=GPIO_NUM_46;
int EncoderLB=GPIO_NUM_3;
int EncoderRA=GPIO_NUM_17;
int EncoderRB=GPIO_NUM_18;

void setup() {
  // put your setup code here, to run once:
  pinMode(EncoderLA,INPUT);
  pinMode(EncoderLB,INPUT);
  pinMode(EncoderRA,INPUT);
  pinMode(EncoderRB,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  printf("%d,%d,%d,%d\n",digitalRead(EncoderLA),digitalRead(EncoderLB),digitalRead(EncoderRA),digitalRead(EncoderRB));
  delay(200);
}

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }