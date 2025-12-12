#ifndef DC_MOTOR_H
#define DC_MOTOR_H

class DCMotor {
    public:
        DCMotor(int forward_pin, int backward_pin, int power_pin);
        void forward_motion();
        void backward_motion();
        void stop_motion();

    private:
        const int _forward_pin, _backward_pin, _power_pin;
};

#endif