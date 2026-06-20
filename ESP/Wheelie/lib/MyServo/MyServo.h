#ifndef MY_SERVO_H
#define MY_SERVO_H

#include <Arduino.h>

class MyServo {
  private:
    int _pin;
    int _min_us;
    int _max_us;
    float _current_angle;
    float _s_start;   // angle when current movement began (for S-curve)
    float _s_end;     // target when current movement began (for S-curve)
    const char* _name;

  public:
    MyServo(const char* name); // Updated constructor
    void setup(int pin, int min_us, int max_us);
    void write(int angle);
    void speedControl(float target_angle, float speed_deg_per_sec);
    // The Debug Function
    void debug();
};

#endif