#ifndef INFRARED_PROXIMITY_SENSOR_H
#define INFRARED_PROXIMITY_SENSOR_H

class InfraredProximitySensor {
    public:
        InfraredProximitySensor(int ir_pin, int threshold);
        bool is_white();

    private:
        const int _ir_pin, _threshold;
        int _get_ir_value();
};

#endif