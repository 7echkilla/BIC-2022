#include "infrared_proximity_sensor.h"
#include <Arduino.h>

InfraredProximitySensor::InfraredProximitySensor(int ir_pin, int threshold) 
: _ir_pin(ir_pin), _threshold(threshold) {
    pinMode(_ir_pin, INPUT);
}

int InfraredProximitySensor::_get_ir_value() {
    return analogRead(_ir_pin);
}

bool InfraredProximitySensor::is_white() {
    int ir_value = _get_ir_value();
    bool is_white = ir_value > _threshold;
    
    return is_white;
}