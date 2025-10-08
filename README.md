Autonomous Robot Navigation System 🤖
📌 Project Overview

This project implements an Autonomous Mobile Robot (TurtleBot) that can navigate its environment intelligently. The robot integrates Arduino C programming, sensors, motor control, and a web-based Bluetooth Low Energy (BLE) controller for manual operation.

It supports three operating modes:

Obstacle Avoidance (using ultrasonic sensor)

Path Following (using IR sensors)

Manual Control (via BLE web app)

The project demonstrates embedded C concepts like loops, conditionals, arrays, functions, structures, interrupts, and modular design.

🗂 Project Files

final_TurtleBot_code.ino
Arduino sketch written in C for robot control.

Implements obstacle avoidance, path following, and manual control.

Uses interrupts, structures, and modular functions.

web_application.html
A responsive web application to control the robot via Bluetooth Low Energy (BLE).

Built using HTML5, CSS3, and JavaScript.

Provides connection interface, D-Pad controls, and command buttons.

autonomus robot navigation system.pptx
Project presentation explaining system architecture, hardware/software requirements, C programming principles, and design flow.

⚙️ Hardware Requirements

Arduino UNO R4 WiFi (with built-in Bluetooth)

L298N Motor Driver

HC-SR04 Ultrasonic Sensor

TCRT5000 IR Line Sensors (x3)

DC Motors with Wheels

Servo Motor

Power Supply: 6 × AA Batteries + Holder

Chassis & Assembly Components

🖥 Software Requirements

Arduino IDE (for uploading .ino code)

Google Chrome (supports Web Bluetooth API for the controller)

VS Code / Any Editor (to edit HTML & Arduino code)

🚀 Setup Instructions
1️⃣ Arduino Code

Open final_TurtleBot_code.ino in Arduino IDE.

Select Arduino UNO R4 WiFi board.

Connect your Arduino via USB.

Upload the sketch.

2️⃣ Web Application

Open web_application.html in Google Chrome.

Click "Connect to BLE Device" and pair with the Arduino’s BLE module.

Use the D-Pad or function buttons to control the robot manually.

🧩 System Architecture

Input Layer: Ultrasonic & IR sensors + Bluetooth user commands

Processing Layer: Arduino executes C code, applies logic, and switches modes

Output Layer: Motors and actuators controlled by L298N driver

🎯 Applications

Warehouse robots for goods movement

Delivery and inspection robots

Educational robotics projects
