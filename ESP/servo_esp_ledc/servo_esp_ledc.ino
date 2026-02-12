
void SetServoAngle(int ServoPin,int angle){
  int dutycycle=map(angle,0,180,0,255);
  ledcWrite(ServoPin, dutycycle);
}
void setup() {
  // put your setup code here, to run once:
  pinMode(35,OUTPUT);
  Serial.begin(115200);
  ledcAttach(14, 50, 4);
  
}

void loop() {
  //SetServoAngle(48,0);
  analogWrite(14,128);
  // put your main code here, to run repeatedly:
  // for(int i=0;i<180;i++){
  //   Serial.println(i);
  //   SetServoAngle(0,i);
  //   delay(100);
  // }
  // for(int i=180;i>0;i--){
     
  //    Serial.println(i);
  //    SetServoAngle(0,i);
  //    delay(100);
  // }
 
}
