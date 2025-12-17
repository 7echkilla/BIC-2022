#ifndef DC_MOTOR_H
#define DC_MOTOR_H

class DCMotor {
    public:
        DCMotor(int forward_pin, int backward_pin, int power_pin, float min_speed, float max_speed);
        void drive_motor(float value);

    private:
        const int _forward_pin, _backward_pin, _power_pin;
        const float _max_speed, _min_speed;
        void _forward_motion(int speed);
        void _backward_motion(int speed);
        void _stop_motion();
};

#endif