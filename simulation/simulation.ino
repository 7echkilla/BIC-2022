#include "analog_joystick.h"
#include "infrared_proximity_sensor.h"
#include "ultrasonic_sensor.h"
#include "dc_motor.h"
#include <Arduino.h>
#include <Servo.h>

// Analog joystick config
#define HORZ_PIN A0
#define VERT_PIN A1
#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 1023
#define DEADZONE 0.12

// IR sensor config
#define IR_R_PIN A2
#define IR_L_PIN A3 
#define IR_THRESHOLD 400

// Ultrasonic sensor config
#define ECHO_PIN 4
#define TRIG_PIN 5

// DC Motor config
#define MOTOR_LF_PIN 12
#define MOTOR_LB_PIN 13
#define MOTOR_RF_PIN 10
#define MOTOR_RB_PIN 9
#define MOTOR_EN_PIN 11
#define MIN_SPEED 30.0
#define MAX_SPEED 255.0

// Servo config
#define SERVO_PIN 3
#define SERVO_MIN 0
#define SERVO_MAX 180

#define BAUD_RATE 9600

AnalogJoystick analog_joystick(HORZ_PIN, VERT_PIN, JOYSTICK_MIN, JOYSTICK_MAX);
InfraredProximitySensor ir_left(IR_L_PIN, IR_THRESHOLD), ir_right(IR_R_PIN, IR_THRESHOLD);
UltrasonicSensor ultrasonic_sensor(TRIG_PIN, ECHO_PIN);
DCMotor motor_left(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_EN_PIN, MIN_SPEED, MAX_SPEED);
DCMotor motor_right(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_EN_PIN, MIN_SPEED, MAX_SPEED);
Servo servo;

void setup() {
    pinMode(HORZ_PIN, INPUT);
    pinMode(VERT_PIN, INPUT);

    pinMode(IR_L_PIN, INPUT);
    pinMode(IR_R_PIN, INPUT);

    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    pinMode(MOTOR_LF_PIN, OUTPUT);
    pinMode(MOTOR_LB_PIN, OUTPUT);
    pinMode(MOTOR_RF_PIN, OUTPUT);
    pinMode(MOTOR_RB_PIN, OUTPUT);
    pinMode(MOTOR_EN_PIN, OUTPUT);
    
    servo.attach(SERVO_PIN);
    Serial.begin(BAUD_RATE);
}

void loop() {
    float x_value = analog_joystick.get_x_value();
    float y_value = analog_joystick.get_y_value();    
    int angle = SERVO_MAX / 2;

    // Tank-like movement for spin
    if (fabs(y_value) < DEADZONE) {
        // Steering angle: 0 (left) or 180 (right)
        if (x_value > 0) {
            servo.write(SERVO_MAX);
        } else if (x_value < 0) {
            servo.write(SERVO_MIN);
        } 

        motor_left.drive_motor(x_value);
        motor_right.drive_motor(-x_value);
    } else {
        // Car-like behaviour (forward/backward + steering)
        angle = constrain(round((x_value + 1) * (SERVO_MAX / 2)), SERVO_MIN, SERVO_MAX);
        servo.write(angle);

        // Calculated left/right motor values based on car-behaviour
        float left_value = constrain(y_value + x_value * fabs(y_value), -1.0, 1.0);
        float right_value = constrain(y_value - x_value * fabs(y_value), -1.0, 1.0);

        // Reversed values for backward-diagonal motions
        if (y_value < 0) {
            float temp = left_value;
            left_value = right_value;
            right_value = temp;
        }

        motor_left.drive_motor(left_value);
        motor_right.drive_motor(right_value);
    }
}
