/*description
author :Yong Sheng Kai
Last modify date: 21/1/2026
Arduino UNO
MT6701 encoder
MG995 Servo
MPU6050 
//FYP 1 DEMO CODE

*/

#include <MPU6050_light.h
MPU6050 mpu(Wire);

int count = 0;         //ppr count
float RPS = 0.0;       //wheel speed
bool ledstate = true;  // led on board to let user know if the program is freeze or not

float error = 0.0, errorsum = 0.0, fpidout = 0.0;
float Kp = 1.0,
      Ki = 0.05,
      Kd = 0.0;
float fsetval = 0.0;  //set angle for motor


float getAngle() {  //read angle from mpu6050
  mpu.update();
  float angle = mpu.getAngleY();
  return angle;
}

int pidformotor() {                //give motor data to run(probally need to return velocity/torque data)
  error = fsetval - (getAngle());  //calculate error from angle set
  errorsum = errorsum + error;
  fpidout = (Kp * error) + (Ki * errorsum);  //PI Implement
  if (fpidout >= 0) {
    if (fpidout > 255) {  //ensure fpidout does not more then 255
      fpidout = 255;
    } else {
      fpidout = fpidout;
    }
  } else if (fpidout < 0) {
    if (fpidout < -255) {  //ensure fpidout does not less then -255
      fpidout = -255;
    } else {
      fpidout = fpidout;
    }
  } else {  //set fpidout =0 if any issue occur
    fpidout = 0;
  }
  return (fpidout);
}
void setup() {
  // put your setup code here, to run once:
  //encoder setup
  pinMode(2, INPUT);                                            //define pin for encoder A pin
  pinMode(3, INPUT);                                            //define pin for Encoder B pin
  pinMode(13, OUTPUT);                                          //led onboard
  attachInterrupt(digitalPinToInterrupt(2), DirCheck, RISING);  // digital pin interrupts
  Serial.begin(9600);

  //mpu6050 setup
  Wire.begin();
  byte status = mpu.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcAccOffsets();

  //timer interrupt setup
  noInterrupts();
  TCCR1A = 0;           //clear all data in tccr1a
  TCCR1B = 0;           //clear all data in tccr1b
  TCCR1B |= B00000100;  //set prescaller to 255
  TIMSK1 |= B00000010;  //enable timer interrupts
  OCR1A = 6250;         // timer overflow value (6250 for 0.1s)
  interrupts();         // enable interrupts
}

void loop() {
  // put your main code here, to run repeatedly:
  //pidformotor();
  Serial.print(count);  //encoder count for motor
  Serial.print(" ");
  Serial.print(RPS);  //calculated Wheel Speed
  Serial.print(" ");
  Serial.print(error);  //error between angle set and current angle of robot
  Serial.print(" ");
  Serial.print(errorsum);  //sum of error between angle set and current angle of robot
  Serial.print(" ");
  Serial.print(pidformotor());  //value to motor for balance back the robot
  Serial.print(" ");
  Serial.println(getAngle());  //angle data
  delay(200);
}

void DirCheck() {             //add count if counter clock wise, otherwise deduct it.
  int Bpin = digitalRead(3);  //read from another encoder pin to check the direction
  if (Bpin == 1) {            // direction CW
    count++;
  } else {  //Direction CCW
    count--;
  }
}

ISR(TIMER1_COMPA_vect) {        //timer interrupts services routins
  TCNT1 = 0;                    //reset timer
  ledstate = !ledstate;         //Invert LED state
  digitalWrite(13, ledstate);   //led onboard
  RPS = (count / 1024.0) * 10;  //count wheel speed
  count = 0;                    //reset encoder count
}
