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

void MyServo::speedControl(float target_angle, float speed_deg_per_sec) {
    // Called once per control loop iteration — NOT a blocking while loop
    float dt_ms   = 100.0f;                                    // your fixed 5 ms timestep
    float maxStep = speed_deg_per_sec * (dt_ms / 1000.0f);
    float error   = target_angle - _current_angle;

    if (fabs(error) < 0.5f) {
        _current_angle = target_angle;                        // snap — fixes int deadband bug
    } else if (error > 0) {
        _current_angle += min(maxStep, error);                // no overshoot
    } else {
        _current_angle -= min(maxStep, -error);
    }

    write(_current_angle);
}

void MyServo::debug() {
    int us = map((int)_current_angle, 0, 180, _min_us, _max_us);
    printf("[%s] Angle: %.2f | Pulse: %d us\n", _name, _current_angle, us);
}