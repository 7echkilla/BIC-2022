#ifndef ANALOG_JOYSTICK_H
#define ANALOG_JOYSTICK_H

class AnalogJoystick {
    public:
        AnalogJoystick(int x_pin, int y_pin, int sel_pin, int max);
        float get_x_value();
        float get_y_value();
        int get_sel_value();

    private:
        int _x_pin, _y_pin, _sel_pin, _max_input;
        int _last_sel_value, _sel_value;
        unsigned long _last_debounce_time, _debounce_delay;
};

#endif