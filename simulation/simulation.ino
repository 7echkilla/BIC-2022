#include "analog_joystick.h"
#include "infrared_proximity_sensor.h"
#include "ultrasonic_sensor.h"
#include <Arduino.h>

// Analog joystick config
#define HORZ_PIN A0
#define VERT_PIN A1
#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 1023

// IR sensor config
#define IR_R_PIN A2
#define IR_L_PIN A3 
#define IR_THRESHOLD 400

// Ultrasonic sensor config
#define ECHO_PIN 4
#define TRIG_PIN 5

#define BAUD_RATE 9600

AnalogJoystick analog_joystick(HORZ_PIN, VERT_PIN);
InfraredProximitySensor ir_sensor_left(IR_L_PIN, IR_THRESHOLD), ir_sensor_right(IR_R_PIN, IR_THRESHOLD);
UltrasonicSensor ultrasonic_sensor(TRIG_PIN, ECHO_PIN);


void setup() {
    pinMode(X_PIN, INPUT);
    pinMode(Y_PIN, INPUT);

    pinMode(IR_L_PIN, INPUT);
    pinMode(IR_R_PIN, INPUT);
    
    Serial.begin(BAUD_RATE);
}

void loop() {
    // int x_value = analog_joystick.get_x_value();
    // int y_value = analog_joystick.get_y_value();

    // Serial.print("X value: ");
    // Serial.print(x_value);
    // Serial.print("Y value: ");
    // Serial.println(y_value);

    // Serial.println(ir_sensor_left.is_black());
    Serial.println(ultrasonic_sensor.)

    delay(100);
}
