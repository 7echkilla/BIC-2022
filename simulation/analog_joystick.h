#ifndef ANALOG_JOYSTICK_H
#define ANALOG_JOYSTICK_H

class AnalogJoystick {
    public:
        AnalogJoystick(int x_pin, int y_pin, int min, int max);
        int get_x_value();
        int get_y_value();

    private:
        int _x_pin, _y_pin, _min, _max;
};

#endif