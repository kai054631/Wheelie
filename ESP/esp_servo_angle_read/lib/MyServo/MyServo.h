#ifndef MY_SERVO_H
#define MY_SERVO_H

#include <Arduino.h>

class MyServo {
  private:
    int _pin;
    int _min_us;
    int _max_us;
    float _current_angle;
    const char* _name; // Add a name to identify which servo is talking

  public:
    MyServo(const char* name); // Updated constructor
    void setup(int pin, int min_us, int max_us);
    void write(int angle);
    void speedControl(int target_angle, float speed);
    
    // The Debug Function
    void debug(); 
};

#endif