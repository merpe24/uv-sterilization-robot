# UV Sterilization Robot

An autonomous UV-C sterilization rover built on ESP32 with WiFi-based 
control and real-time obstacle avoidance.

![UV Robot](https://github.com/merpe24/uv-sterilization-robot/blob/main/IMG_3509.png)

---

## Overview

The UV Robot is a WiFi-controlled rover designed to perform UV-C light 
sterilization of indoor spaces. It operates entirely through a browser-based 
interface hosted on the ESP32 itself — no app installation required. Users 
can control the robot from any smartphone or laptop connected to the robot's 
WiFi access point.

This project was built as part of the Sensors, Signaling, and Actuators for 
Robotics Projects course at the International School of Engineering, 
Chulalongkorn University.

---

## Features

- **Manual Mode** — Control the robot via directional buttons on a web 
  browser with adjustable speed
- **Autonomous Mode** — Real-time obstacle avoidance using ultrasonic and 
  proximity sensor fusion
- **Replay Mode** — Record a driven path and replay it autonomously
- **Zero-install control interface** — Web server hosted directly on ESP32, 
  accessible from any browser

---

## Hardware

| Component | Role |
|---|---|
| ESP32 Microcontroller | Central processing unit + WiFi access point + web server |
| DRV8825 Stepper Motor Drivers (x2) | Precise motor control for left and right wheels |
| HC-SR04 Ultrasonic Sensor | Forward obstacle detection (2cm–400cm range) |
| VCNL4040 Proximity Sensors (x2) | Left and right obstacle detection |
| UV Light Module (simulated with blue LEDs) | UV-C sterilization payload |

---

## System Architecture

The robot operates across 4 integrated subsystems:

1. **ESP32 Web Server** — Creates a WiFi access point (`UVRobot-AP`) and 
   hosts a single-page control interface at `192.168.4.1`
2. **Motor Control** — Two stepper motors via DRV8825 drivers on GPIO pins 
   18/19 (left) and 23/27 (right)
3. **Sensor Array** — Continuous polling of 3 sensors to detect obstacles 
   within 20cm threshold
4. **UV Module** — Activated automatically during operation to sterilize 
   the floor surface

---

## How It Works

### Autonomous Obstacle Avoidance
The robot continuously polls all three sensors. When any sensor detects 
an obstacle within 20cm, the robot stops and turns toward the direction 
with greater clearance. This runs entirely on the ESP32 with no external 
computation.

### Web Interface
The ESP32 hosts a responsive HTML/CSS/JS control page accessible from 
any browser. The interface visually disables directional controls when 
autonomous mode is active to prevent conflicting inputs.

---

## Results

- Manual mode executed directional commands with low latency over local WiFi
- Autonomous obstacle avoidance was consistent across multiple test runs
- Replay mode successfully reproduced recorded paths in real time
- Web interface loaded correctly on Android, iOS, and laptop browsers

---

## Demo

▶️ [Watch the robot in action](https://youtu.be/0Mvx4flVzoA)

![UV Robot Demo](https://github.com/merpe24/uv-sterilization-robot/blob/main/ROBO%20UV.gif.gif)

## Limitations & Future Work

- UV-C light was simulated with blue LEDs due to budget constraints — 
  a real UV-C source can be connected via 12V power
- Currently limited to flat indoor surfaces
- Navigation is reactive (rule-based) — a future version could implement 
  full coverage path planning using ROS 2 and SLAM
- Sensor suite could be expanded with a camera for vision-based navigation

---

## Built With

- ESP32 Arduino Framework
- HTML / CSS / JavaScript (web interface)
- C++ (embedded control logic)

---

## Team

Built by Group 7, ISE Chulalongkorn University  
Course: Sensors, Signaling, and Actuators for Robotics Projects

---

## What I Learned

- Integrating multiple sensor types (ultrasonic + proximity) for robust 
  obstacle detection
- Hosting a functional web server on a microcontroller
- Designing reactive autonomous behavior on resource-constrained hardware
- Hardware assembly and circuit design for a complete mobile robot system
