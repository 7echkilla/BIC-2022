#include "analog_joystick.h"
#include <Arduino.h>

AnalogJoystick::AnalogJoystick(int x_pin, int y_pin, int sel_pin, int max_input) 
: _x_pin(x_pin), _y_pin(y_pin), _sel_pin(sel_pin), _max_input(max_input) {
    pinMode(_x_pin, INPUT);
    pinMode(_y_pin, INPUT);
    pinMode(_sel_pin, INPUT);
    _last_debounce_time = 0;
    _debounce_delay = 50;   // Delay to register debounce in ms
    _last_sel_value = HIGH; // Default state for select pin (unpressed)
}

// Convert 0/1023 analog input to -1/1
float AnalogJoystick::get_x_value() {
    float x_value = (analogRead(_x_pin) - (_max_input / 2)) / (_max_input / 2);
    return x_value;
}

float AnalogJoystick::get_y_value() {
    float y_value = (analogRead(_y_pin) - (_max_input / 2)) / (_max_input / 2);
    return y_value;
}

int AnalogJoystick::get_sel_value() {
    int current_sel_value = digitalRead(_sel_pin);

    // Reset timer on state change
    if (current_sel_value != _last_sel_value) {
        _last_debounce_time = millis();
    }

    // Check if sufficient time has elapsed
    if ((millis() - _last_debounce_time) > _debounce_delay) {
        if (current_sel_value != _sel_value) {
            _sel_value = current_sel_value;
        }
    }

    // Always update last registered value
    _last_sel_value = current_sel_value;
    return _sel_value;
}