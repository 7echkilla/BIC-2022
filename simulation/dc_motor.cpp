#include "dc_motor.h"
#include <Arduino.h>

DCMotor::DCMotor(int forward_pin, int backward_pin, int power_pin)
    : _forward_pin(forward_pin), _backward_pin(backward_pin), _power_pin(power_pin) {
}

void DCMotor::forward_motion() {
    digitalWrite(_forward_pin, HIGH);
    digitalWtite(_backward_pin, LOW);
}

void DCMotor::backward_motion() {
    digitalWrite(_forward_pin, LOW);
    digitalWtite(_backward_pin, HIGH);
}

void DCMotor::stop_motion() {
    digitalWrite(_forward_pin, LOW);
    digitalWtite(_backward_pin, LOW);
}