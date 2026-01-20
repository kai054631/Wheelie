#include <Servo.h>

Servo myservo;
void setServo(bool servo){
  if(servo==1){
    //state up
    myservo.write(90);
    Serial.println("Stand up");
  }else if(servo==0){
    myservo.write(50);
    Serial.println("Squad Down");
  }
}
void setup() {
  // put your setup code here, to run once:
  myservo.attach(6);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  setServo(1);
  delay(1000);
  setServo(0);

}
