#include "analog_joystick.h"
#include <Arduino.h>

AnalogJoystick::AnalogJoystick(int x_pin, int y_pin, int min, int max)
    : _x_pin(x_pin), _y_pin(y_pin), _min(min), _max(max) {
}

int AnalogJoystick::get_x_value() {
    int x_value = map(analogRead(_x_pin), _min, _max, -1, 1);
    return x_value;
}

int AnalogJoystick::get_y_value() {
    int y_value = map(analogRead(_x_pin), _min, _max, -1, 1);
    return y_value;
}