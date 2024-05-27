# Line Follower Robot with PID Control

This repository contains the code and instructions for building a line follower robot using an Arduino Uno, an L298N motor driver, and a 5-array IR sensor. The robot uses a PID (Proportional-Integral-Derivative) controller to follow a line with high precision.

## Table of Contents
- [Introduction](#introduction)
- [Components](#components)
- [Circuit Diagram](#circuit-diagram)
- [Setup Instructions](#setup-instructions)
- [Code Explanation](#code-explanation)
- [PID Tuning](#pid-tuning)
- [Usage](#usage)
- [License](#license)

## Introduction
A line follower robot is an autonomous vehicle that follows a predefined path. This project uses an array of IR sensors to detect the line and a PID controller to adjust the motor speeds for precise line tracking.

## Components
- Arduino Uno
- L298N Motor Driver
- 5-array IR Sensors
- 2 DC Motors (600 RPM wheels)
- Chassis
- Power supply (battery pack)
- Connecting wires

## Circuit Diagram
The circuit diagram for this project is as follows:
![Circuit Diagram](https://github.com/mostafayossrii/LFR/blob/main/OldVersion/Arduino-Line-Following-Robot-1024x922.png)

[Include a link to the circuit diagram image or a Fritzing diagram here]
## Setup Instructions

1. Wiring:
    - Connect the IR sensors to the analog pins of the Arduino (A0 to A4).
    - Connect the L298N motor driver to the digital pins of the Arduino for motor control.
    - Connect the motors to the output pins of the L298N motor driver.
    - Provide power to the Arduino and the motor driver using a suitable power source.

2. Installing Arduino IDE:
    - Download and install the Arduino IDE from [here](https://www.arduino.cc/en/Main/Software).

3. Library Dependencies:
    - Ensure you have the required libraries installed in your Arduino IDE. This project doesn't require any external libraries, but it's good practice to check for updates to the default libraries.

## Code Explanation
The code for the line follower robot includes the following key components:

- Sensor Reading: Reads values from the IR sensors to detect the position of the line.
- PID Control: Implements a PID controller to compute the error and adjust motor speeds accordingly.
- Motor Control: Sends PWM signals to the motor driver to control the speed and direction of the motors.

## PID Tuning
Tuning the PID controller is crucial for optimal performance. Start with small values for Kp, Ki, and Kd, and gradually increase them while observing the robot's behavior.

- Kp (Proportional): Adjusts the speed proportionally to the error.
- Ki (Integral): Eliminates steady-state error by integrating the error over time.
- Kd (Derivative): Reduces overshooting by considering the rate of change of the error.

## Usage
1. Upload the code to the Arduino Uno using the Arduino IDE.
2. Place the robot on the track.
3. Turn on the power supply.
4. The robot should start following the line automatically.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
