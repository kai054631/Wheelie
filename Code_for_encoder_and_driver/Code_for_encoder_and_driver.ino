#include <SimpleFOC.h>

//Encoder Part
Encoder encoder = Encoder(2, 3, 1024); //encoder(a,b,ppr Value)

//Motor n Driver Settings 
BLDCMotor motor = BLDCMotor(7,3.2,220); //(pole pair,phase resistance,kv Value)
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10,9, 8);  //(in1,in2,in3,enPin)

void doA(){
  encoder.handleA();
}
void doB(){
  encoder.handleB();
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  //Encoder Part
  encoder.init(); //initate encoder
  encoder.enableInterrupts(doA, doB);// enable interupt for encoder

  //Motor n Driver Settings 
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 3;

  //Error Message if initial fail
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  
  // enable driver
  driver.enable();
  Serial.println("Driver ready!");
  _delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Encoder
  
  encoder.update();// read sensor and update the internal variables
  Serial.print("Rad: ");// display the rad to the terminal
  Serial.println(encoder.getAngle()); //1 rotation =6.28(2*Pi)
  
  //Driver n Motor parts
  // setting pwm
    // phase A: 3V
    // phase B: 6V
    // phase C: 5V
    driver.setPwm(0.75,1.5,1.25);//not sure what this mean
    
  
 

}
