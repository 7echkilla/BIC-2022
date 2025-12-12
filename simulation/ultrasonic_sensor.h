#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

class UltrasonicSensor {
    public:
        UltrasonicSensor(int trig_pin, int echo_pin);
        int get_distance();

    private:
        const int _trig_pin, _echo_pin;
        const float factor = 0.0343 / 2;
};

#endif