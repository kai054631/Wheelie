#include "MyServo.h"

MyServo::MyServo(const char* name) {
    _name = name;
    _current_angle = 0;
    _s_start = 0;
    _s_end   = 0;
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

void MyServo::speedControl(float targetAngle, float speed_deg_per_sec) {
    const float dt = 0.1f;  // TaskServoCode calls every 100 ms
    float maxStep = speed_deg_per_sec * dt;
    float error   = targetAngle - _current_angle;

    if (fabs(error) < 0.5f) {
        _current_angle = targetAngle;
    } else if (error > 0) {
        _current_angle += min(maxStep, error);
    } else {
        _current_angle -= min(maxStep, -error);
    }

    write(_current_angle);
}

void MyServo::debug() {
    int us = map((int)_current_angle, 0, 180, _min_us, _max_us);
    printf("[%s] Angle: %.2f | Pulse: %d us\n", _name, _current_angle, us);
}