#include "dc_motor.h"
#include <Arduino.h>

DCMotor::DCMotor(int forward_pin, int backward_pin, int power_pin,float min_speed, float max_speed) 
: _forward_pin(forward_pin), _backward_pin(backward_pin), _power_pin(power_pin), _min_speed(min_speed), _max_speed(max_speed) {
    pinMode(_forward_pin, OUTPUT);
    pinMode(_backward_pin, OUTPUT);
    pinMode(_power_pin, OUTPUT);
}

void DCMotor:: drive_motor(float value) {
    float speed = constrain(fabs(value) * _max_speed, 0, _max_speed);

    if (speed < _min_speed) {
        _stop_motion();
    } else if (value > 0) {
        _forward_motion(speed);
    } else {
        _backward_motion(speed);
    }
}

void DCMotor::_forward_motion(int speed) {
    digitalWrite(_forward_pin, HIGH);
    digitalWrite(_backward_pin, LOW);
    analogWrite(_power_pin, speed);
}

void DCMotor::_backward_motion(int speed) {
    digitalWrite(_forward_pin, LOW);
    digitalWrite(_backward_pin, HIGH);
    analogWrite(_power_pin, speed);
}

void DCMotor::_stop_motion() {
    digitalWrite(_forward_pin, LOW);
    digitalWrite(_backward_pin, LOW);
}