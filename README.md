# Line-following RC Car
This project for [Build It Challenge 2022](https://www.facebook.com/events/710362346658275?ref=newsfeed) involves a RC car controlled by two Arduino boards (Master and Slave), which communicate via Bluetooth. The system operates in two modes:
- **Manual Mode**: Controlled by an analog joystick connected to the Master Arduino.
- **Autonomous Mode**: The car follows a black tape on a white surface using IR sensors and avoids obstacles using a proximity sensor.

The Master Arduino controls the overall mode (manual or automatic) and sends joystick data to the Slave Arduino when in manual mode. The Slave Arduino uses the joystick data to drive the motors and control the steering (servo), and in automatic mode, it uses sensors to make decisions autonomously.

## Project Strucure
/lane-cruiser\
│\
├── /master # Controller code (master arduino)\
│ ├── [analog_joystick.h](./master/analog_joystick.h) # Header file for joystick class\
│ ├── [analog_joystick.cpp](./master/analog_joystick.cpp) # Source code for joystick logic\
│ └── [master.ino](./master/master.ino) # Main code for the master arduino\
│\
├── /simulation\
│ ├── [analog_joystick.h](./master/analog_joystick.h) # Header file for joystick class\
│ ├── [analog_joystick.cpp](./master/analog_joystick.cpp) # Source code for joystick logic\
│ ├── [dc_motor.h](./simulation/dc_motor.h) # Header file for motor class\
│ ├── [dc_motor.cpp](./simulation/dc_motor.cpp) # Source file for motor logic\
│ ├── [infrared_proximity_sensor.h](./simulation/infrared_proximity_sensor.h) # Header file for ir sensor class\
│ ├── [infrared_proximity_sensor.cpp](./simulation/infrared_proximity_sensor.cpp) # Source code for ir sensor logic\
│ ├── [README.md](./simulation/README.md) # Documentation for CirkitDesigner project\
│ ├── [simulation.ino](./simulation/simulation.ino) # Main code for the simulation\
│ ├── [ultrasonic_sensor.h](./simulation/ultrasonic_sensor.h) # Header file for proximity sensor class\
│ └── [ultrasonic_sensor.cpp](./simulation/ultrasonic_sensor.cpp) # Source code for proximity sensor class\
│\
├── /slave # RC car code (slave arduino)\
│ ├── [dc_motor.h](./slave/dc_motor.h) # Header file for motor class\
│ ├── [dc_motor.cpp](./slave/dc_motor.cpp) # Source file for motor logic\
│ ├── [infrared_proximity_sensor.h](./slave/infrared_proximity_sensor.h) # Header file for ir sensor class\
│ ├── [infrared_proximity_sensor.cpp](./slave/infrared_proximity_sensor.cpp) # Source code for ir sensor logic\
│ ├── [slave.ino](./slave/slave.ino) # Main code for the slave arduino\
│ ├── [ultrasonic_sensor.h](./slave/ultrasonic_sensor.h) # Header file for proximity sensor class\
│ └── [ultrasonic_sensor.cpp](./slave/ultrasonic_sensor.cpp) # Source code for proximity sensor class\
│\
└── [README.md](README.md)

The files are mainly organised as follows:
- **analog_joystick.cpp**: This file contains the implementation for handling the analog joystick input. It simulates reading the joystick’s X and Y axis values and processes them into control signals for the RC car’s movement in manual mode.
- **analog_joystick.h**: The header file for the analog_joystick.cpp implementation. It defines the functions and data structures necessary for handling joystick input.
- **dc_motor.cpp**: This file contains the code to simulate the behavior of the DC motors used in the RC car. It includes functions to control the motor's speed and direction.
- **dc_motor.h**: The header file for the dc_motor.cpp implementation. It defines the functions and constants related to motor control.
- **infrared_proximity.cpp**: This file simulates the infrared proximity sensor used for obstacle detection. The sensor is used in automated mode to follow lines on the ground.
- **infrared_proximity.h**: The header file for the infrared_proximity.cpp implementation. It defines the proximity sensor's interface for reading line-following (colour) status.

## Prerequisites
To get started with the project, ensure you have the following:

### Hardware Components
- 2x Arduino boards ([Arduino Uno](https://core-electronics.com.au/arduino-uno-r3.html) or similar)
- 2x Bluetooth modules ([HC-05](https://core-electronics.com.au/bluetooth-module-hc-05.html))
- 1x [Analog joystick](https://core-electronics.com.au/analog-2-axis-thumb-joystick-with-select-button-breakout-board.html)
- 1x [Servo](https://core-electronics.com.au/feetech-1-5kg-continuous-rotation-servo-fs90r.html) (for steering control)
- 2x [DC motor](https://core-electronics.com.au/dc-motor-with-jumper-wire.html) (for driving the wheels)
- 2x IR sensors ([LDR](https://www.makerstore.com.au/product/mb-elc-sen-ldr-mod/?srsltid=AfmBOopRZ7Rsb3HpoevAYln54rGLOHa5YdCcpsOy_uxabnCI0FHyX5HMPKc))
- 1x Proximity sensor ([HC-SR04](https://core-electronics.com.au/hc-sr04-ultrasonic-module-distance-measuring-sensor.html))
- 1x [Motor Controller](https://core-electronics.com.au/2a-dual-motor-controller.html)
- 1x RC car chassis (see [article](https://www.hackster.io/mertarduino/wireless-arduino-motor-driver-and-joystick-controller-5e4a55) for inspiration)

See [article](https://howtomechatronics.com/tutorials/arduino/how-to-configure-pair-two-hc-05-bluetooth-module-master-slave-commands/) to configure bluetooth modules in AT mode to enable communication.

### Software Requirements
- [Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE) (or similar) to compile and upload code to the boards
- [CirkitDesigner](https://app.cirkitdesigner.com/) (optional) to simulate car behaviour as in /[simulation](./simulation/)

## Drive Controls
The project employs a combination of car and tank-like drive using a single joystick with x/y = [-1, 1]:
```cpp
// tank-like control (in-place spin)
float left = y + x;
float right = y - x; 

// car-like control
float left = y + x * abs(y);
float right = y - x * abs(y);

// car+tank mode
float spin_weight = 0.0 // 0 = car, 1 = tank
float left = y + x * (spin_weight + (1 - spin_weight) * abs(y));
float right = y - x * (spin_weight + (1 - spin_weight) * abs(y));
```
