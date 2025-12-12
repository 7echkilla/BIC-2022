#include "infrared_proximity_sensor.h"
#include <Arduino.h>

InfraredProximitySensor::InfraredProximitySensor(int ir_pin, int threshold)
    : _ir_pin(ir_pin), _threshold(threshold) {
}

int InfraredProximitySensor::_get_ir_value() {
    return analogRead(_ir_pin);
}

bool InfraredProximitySensor::is_black() {
    int ir_value = _get_ir_value();
    bool is_black = ir_value < _threshold;
    
    return is_black;
}