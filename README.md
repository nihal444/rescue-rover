# Rescue Rover

![Project Overview](assets/rover-diagram.png)
<!-- If 'assets/rover-diagram.png' does not exist, update the filename to match an actual image in the assets folder. -->

A cross-platform system to control a remote car using a Flutter app (with on-screen joystick/buttons) via Bluetooth and WiFi, communicating with an ESP32-based car and an ESP32-CAM for video streaming and face tracking.

## Features

- **Joystick Control**: Intuitive on-screen joystick for precise car movement.
- **Dual Connectivity**: Control via Bluetooth or WiFi.
- **Real-time Response**: Immediate feedback to joystick movements.
- **Adjustable Sensitivity**: Fine-tune control responsiveness.
- **ESP32 Integration**: Seamless communication with ESP32-based car hardware.
- **Video Streaming**: Live video from ESP32-CAM.
- **Face Tracking**: Automatic car movement based on face detection.

## System Overview

- **Flutter App**: Mobile/web app for user control.
- **ESP32-DEV**: Receives commands via Bluetooth/WiFi, controls motors and sensors.
- **ESP32-CAM**: Streams video and sends face tracking data to ESP32-DEV.

## Getting Started

### 1. Flutter App

- Located in `flutter_app/`
- Controls the car via Bluetooth or WiFi.
- See `flutter_app/README.md` for detailed setup and usage.

### 2. ESP32-DEV Firmware

- File: `facetrack/gyrocar_esp32.ino`
- Receives joystick and face tracking data.
- Controls motors, servo, and sensors.

### 3. ESP32-CAM Firmware

- File: `facetrack/videostream.ino`
- Streams video and sends face tracking/motor commands to ESP32-DEV.

## Hardware Connections

**ESP32-DEV — L298N Motor Driver**
- GPIO15 (MOTOR_A_IN1) → IN1
- GPIO26 (MOTOR_A_IN2) → IN2
- GPIO5  (MOTOR_B_IN3) → IN3
- GPIO18 (MOTOR_B_IN4) → IN4
- GPIO4  (MOTOR_A_ENA) → ENA (PWM)
- GPIO19 (MOTOR_B_ENB) → ENB (PWM)

**ESP32-DEV — HC-SR04 Ultrasonic Sensor**
- GPIO13 (TRIG_PIN) → Trig
- GPIO12 (ECHO_PIN) → Echo

**ESP32-DEV — Status LEDs**
- GPIO2  → LED_STATUS
- GPIO27 → LED_WIFI
- GPIO14 → LED_BT

**ESP32-DEV — ESP32-CAM UART**
- GPIO16 (RXD1) ← ESP32-CAM TXD2 (GPIO1)
- GND shared

**ESP32-CAM**
- Streams video at `/`
- Accepts control commands at `/control` (port 81)

**Power**
- ESP32 5V → sensor VCC & L298N logic VCC
- External battery → L298N 12V motor VCC
- Shared GND

## Usage

1. Flash `gyrocar_esp32.ino` to ESP32-DEV and `videostream.ino` to ESP32-CAM.
2. Connect hardware as described above.
3. Run the Flutter app on your device.
4. Connect via Bluetooth or WiFi and control the car.

## References

- [Flutter documentation](https://docs.flutter.dev/)
- [ESP32-CAM Video Streaming](https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/)

---
For more details, see `flutter_app/README.md`.
