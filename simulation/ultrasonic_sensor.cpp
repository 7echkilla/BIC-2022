#include "ultrasonic_sensor.h"
#include <Arduino.h>

UltrasonicSensor::UltrasonicSensor(int trig_pin, int echo_pin) 
: _trig_pin(trig_pin), _echo_pin(echo_pin) {
    pinMode(_echo_pin, INPUT);
    pinMode(_trig_pin, OUTPUT);
}

int UltrasonicSensor::get_distance() {
    // 10 ms pulse to trigger sensor
    digitalWrite(_trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig_pin, LOW);

    // Calculate distance (duration in ms to cm)
    long duration = pulseIn(_echo_pin, HIGH);
    int distance = duration * factor;

    return distance;
}