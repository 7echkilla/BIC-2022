# Line-following RC Car
This project for [Build It Challenge 2022](https://www.facebook.com/events/710362346658275?ref=newsfeed) involves a RC car controlled by two Arduino boards (Master and Slave), which communicate via Bluetooth. The system operates in two modes:
- **Manual Mode**: Controlled by an analog joystick connected to the Master Arduino.
- **Autonomous Mode**: The car follows a black tape on a white surface using IR sensors and avoids obstacles using a proximity sensor.

## Features
- **Manual Mode**: The car can be controlled via an analog joystick (X and Y axis inputs) connected to the Master Arduino.
- **Autonomous Mode**: The car follows a black tape on a white surface and avoids obstacles using:
  - IR sensors on either side of the car to maintain its position relative to the tape.
  - A front-facing proximity sensor to avoid obstacles.

## Components
- 2x Arduino boards (Master and Slave)
- Bluetooth module (HC-05/HC-06)
- Analog joystick
- Servo (for steering control)
- DC motor (for driving the wheels)
- 2x IR sensors (for tape detection)
- Proximity sensor (for obstacle avoidance)
- RC car chassis

## Communication Protocol
- The Master Arduino sends joystick data (X and Y analog values) over Bluetooth to the Slave Arduino.
- The Slave Arduino receives this data and controls the RC car’s movement (steering and motor).
- In Autonomous Mode, the Slave Arduino uses data from IR sensors to follow the black tape and the proximity sensor to avoid obstacles.

## Project Structure
AutoRC/\
│\
├── master/\
│ ├── master.ino # Arduino Master Code\
│ └── README.md # Documentation for Master\
│\
├── slave/\
│ ├── slave.ino # Arduino Slave Code\
│ └── README.md # Documentation for Slave\
│\
├── docs/\
│ └── wiring_diagrams/ # Circuit schematics and wiring diagrams\
│\
└── README.md\

## Drive Controls
For single joystick, where x/y = [-1, 1]:
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