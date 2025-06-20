# GyroCar

A Flutter app that controls a remote car using an on-screen buttons on flutter app through Bluetooth and also has WiFi communication with an ESP32.

## Features

- **Joystick Control**: Intuitive on-screen joystick for precise car movement control
- **Dual Connectivity**: Connect via Bluetooth or WiFi
- **Real-time Control**: Immediate response to joystick movements
- **Adjustable Sensitivity**: Fine-tune control responsiveness
- **ESP32 Integration**: Seamless communication with ESP32-based car hardware

## Getting Started

This project uses Flutter for the mobile app and Arduino/ESP32 for the car's embedded system.

For help getting started with Flutter development, view the
[online documentation](https://docs.flutter.dev/), which offers tutorials,
samples, guidance on mobile development, and a full API reference.

## Connecting Android App to ESP32 via Bluetooth

1. Firmware
   - Ensure ESP32 runs the `gyrocar_esp32.ino` with `BluetoothSerial`.
   - ESP advertises under name `"GyroCar"`.

2. AndroidManifest
   - Grant `BLUETOOTH`, `BLUETOOTH_ADMIN`, and `ACCESS_FINE_LOCATION` permissions.

3. Runtime Permission
   - Request location permission in Flutter (e.g. via `permission_handler`).

4. Scanning & Connecting

   ```dart
   final devices = await provider.scanBluetoothDevices();
   await provider.connectBluetooth(devices.first.address);
   ```

5. Data Exchange
   - Send JSON‐formatted lines from Flutter using joystick coordinates:
     `connectionService.sendData('{"x":1.0,"y":0.0,"z":0.0}\n');`
   - On ESP32, read in `handleBluetoothData()` and parse for motor control.

6. Disconnect

   ```dart
   await provider.disconnect();
   ```

## Hardware Connections

ESP32  —  L298N Motor Driver
- GPIO15 (MOTOR_A_IN1) → IN1
- GPIO26 (MOTOR_A_IN2) → IN2
- GPIO5  (MOTOR_B_IN3) → IN3
- GPIO18 (MOTOR_B_IN4) → IN4
- GPIO4  (MOTOR_A_ENA) → ENA (PWM channel 0)
- GPIO19 (MOTOR_B_ENB) → ENB (PWM channel 1)

ESP32  —  HC-SR04 Ultrasonic Sensor
- GPIO13 (TRIG_PIN) → Trig
- GPIO12 (ECHO_PIN) → Echo

ESP32  —  Status LEDs (active HIGH)
- GPIO2  → LED_STATUS (on-board)
- GPIO27 → LED_WIFI
- GPIO14 → LED_BT

Power & Ground
- ESP32 5V → sensor VCC & L298N 5V logic VCC
- External motor battery → L298N 12V motor VCC
- Shared GND between ESP32, sensors, and L298N
