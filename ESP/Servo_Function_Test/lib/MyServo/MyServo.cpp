#include "MyServo.h"

MyServo::MyServo(const char* name) {
    _name = name;
    _current_angle = 0;
}

void MyServo::setup(int pin, int min_us, int max_us) {
    _pin = pin;
    _min_us = min_us;
    _max_us = max_us;
    ledcAttach(_pin, 50, 12);
}

void MyServo::write(int angle) {
    _current_angle = angle;
    int us = map(angle, 0, 180, _min_us, _max_us);
    int pulse = (us * 4095) / 20000;
    ledcWrite(_pin, pulse);
}

void MyServo::speedControl(int target_angle, float speed) {
    while(abs(_current_angle - target_angle) > 0.5) {
        if(_current_angle < target_angle) _current_angle += speed;
        else _current_angle -= speed;

        int us = map((int)_current_angle, 0, 180, _min_us, _max_us);
        int pulse = (us * 4095) / 20000;
        ledcWrite(_pin, pulse);
        delay(20);
    }
    write(target_angle);
}

void MyServo::debug() {
    int us = map((int)_current_angle, 0, 180, _min_us, _max_us);
    printf("[%s] Angle: %.2f | Pulse: %d us\n", _name, _current_angle, us);
}