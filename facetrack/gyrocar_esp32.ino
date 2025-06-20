/*
  GyroCar ESP32 Firmware

  Connect ESP32-CAM's TXD2 (GPIO 1) to ESP32-DEV's RXD1 (GPIO 16)
  Connect ESP32-CAM's GND to ESP32-DEV's GND
  
  This firmware allows an ESP32 to receive joystick data from a mobile app
  via either WiFi or Bluetooth, and control DC motors accordingly.
  
  Modified to receive motor control data from ESP32-CAM for face tracking
*/

#include <BluetoothSerial.h> // For Bluetooth connectivity
#include <Wire.h>            // I2C for other sensors (if any)

// Check if Bluetooth is properly supported
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Define UART pins for communication with ESP32-CAM
#define RXD1 16  // ESP32 GPIO pin for receiving data from ESP32-CAM
#define TXD1 17  // ESP32 GPIO pin for sending data to ESP32-CAM (if needed)

// Motor control pins for L298N
#define MOTOR_A_IN1 15  // Input 1 for motor A direction
#define MOTOR_A_IN2 26  // Input 2 for motor A direction
#define MOTOR_B_IN3 5   // Input 3 for motor B direction
#define MOTOR_B_IN4 18  // Input 4 for motor B direction
#define MOTOR_A_ENA 4   // Enable pin for motor A (PWM)
#define MOTOR_B_ENB 19  // Enable pin for motor B (PWM)

// Servo motor control
#define SERVO_PIN 32    // Servo control pin (GPIO 32)
#define SERVO_CHANNEL 2 // PWM channel for servo (different from motors)
#define SERVO_FREQ 50   // 50Hz PWM frequency for servo
#define SERVO_RESOLUTION 16 // 16-bit resolution for smoother control
#define SERVO_MIN_DUTY 1638  // Minimum pulse width (0.5ms at 16-bit resolution)
#define SERVO_MAX_DUTY 8192  // Maximum pulse width (2.5ms at 16-bit resolution)
#define SERVO_CENTER 4915    // Center position (1.5ms at 16-bit resolution)

// Servo scanning parameters
#define SERVO_SCAN_STEP 50   // Step size for servo scanning
#define SERVO_SCAN_DELAY 50  // Delay between steps in ms
#define SERVO_MIN_ANGLE 1800 // Minimum scanning angle
#define SERVO_MAX_ANGLE 8000 // Maximum scanning angle

// HC-SR04 Ultrasonic Sensor pins
#define TRIG_PIN 13     // Trigger pin
#define ECHO_PIN 12     // Echo pin

// Status LEDs
#define LED_STATUS 2    // Built-in LED for status
#define LED_WIFI   2   // LED for WiFi connection status
#define LED_BT     14   // LED for Bluetooth connection status

// PWM properties
#define PWM_FREQ 5000   // PWM frequency
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define PWM_A_CHANNEL 0  // PWM channel for motor A
#define PWM_B_CHANNEL 1  // PWM channel for motor B

// Obstacle avoidance parameters
#define MIN_DISTANCE 20 // Minimum distance in cm before stopping/avoiding
#define AVOID_DURATION 1000 // Duration of avoidance maneuver in ms

// Connection status
bool btConnected = false;

// Current motor values
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
bool leftMotorForward = true;
bool rightMotorForward = true;

// Servo motor state
int servoPosition = SERVO_CENTER;
bool servoContinuousMode = true;  // Start in continuous scanning mode
bool servoDirectionUp = true;     // Direction of servo sweep
bool faceDetected = false;        // Flag for face detection status

// Obstacle avoidance state
bool obstacleDetected = false;
unsigned long obstacleDetectedTime = 0;
bool avoidanceManeuverActive = false;

// Create Bluetooth Serial object
BluetoothSerial SerialBT;

// Last activity timestamp
unsigned long lastActivityTime = 0;
const unsigned long TIMEOUT_MS = 3000; // 3 seconds timeout

// Ultrasonic sensor measurements
long duration;
float distance;
unsigned long lastUltrasonicReadTime = 0;
const unsigned long ULTRASONIC_INTERVAL = 100; // Read every 100ms

// Servo scan timestamps
unsigned long lastServoMoveTime = 0;

// Flag to indicate face tracking mode
bool faceTrackingMode = false;

// Manual control override flags
bool manualControlActive = false;
unsigned long lastBluetoothCommandTime = 0;
const unsigned long BLUETOOTH_TIMEOUT_MS = 2000; // 2 seconds timeout for Bluetooth commands

// System mode (controlled by Flask app via ESP32-CAM)
bool systemAutoMode = false;  // This comes from Flask app via ESP32-CAM

void setup() {
  // Initialize Serial port for debugging
  Serial.begin(115200);
  Serial.println("GyroCar ESP32 Firmware Starting...");
  
  // Initialize UART communication with ESP32-CAM
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial.println("UART connection to ESP32-CAM initialized");
  
  // Initialize status LEDs
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_BT, OUTPUT);
  
  // Initialize motor control pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  
  // Configure PWM for motor speed control
  ledcSetup(PWM_A_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_B_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_ENA, PWM_A_CHANNEL);
  ledcAttachPin(MOTOR_B_ENB, PWM_B_CHANNEL);
  
  // Initialize servo motor
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  
  // Center the servo at startup
  setServoPosition(SERVO_CENTER);
  Serial.println("Servo initialized at center position");
  
  // Initialize HC-SR04 ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Stop motors initially
  stopMotors();
  
  // Start Bluetooth with the name "GyroCar"
  SerialBT.begin("GyroCar");
  // Register Bluetooth connect/disconnect callbacks
  SerialBT.register_callback([](esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    if (event == ESP_SPP_SRV_OPEN_EVT) {
      btConnected = true;
      digitalWrite(LED_BT, HIGH);
      Serial.println("Bluetooth Connected");
    } else if (event == ESP_SPP_CLOSE_EVT) {
      btConnected = false;
      digitalWrite(LED_BT, LOW);
      Serial.println("Bluetooth Disconnected");
      stopMotors();
    }
  });
  
  // // Initialize BMP280 sensor on ESP32 I2C (SDA=21, SCL=22)
  // Wire.begin(23, 25);
  // if (!bmp.begin(0x76)) {
  //   Serial.println("❌ BMP280 init failed. Check wiring or I2C address!");
  //   while (1) delay(100);  // halt
  // } else {
  //   Serial.println("✅ BMP280 initialized");
  // }

  // All systems initialized
  blinkLED(LED_STATUS, 3, 200); // Blink status LED 3 times
}

void loop() {
  // Check for manual control timeout only if system is in manual mode
  if (manualControlActive && !systemAutoMode && (millis() - lastBluetoothCommandTime > BLUETOOTH_TIMEOUT_MS)) {
    manualControlActive = false;
    Serial.println("Manual control timeout - clearing Bluetooth override");
  }

  // Handle incoming Bluetooth data and commands (ONLY when system is in manual mode)
  if (SerialBT.available() && !systemAutoMode) {
    handleBluetoothData();
  } else if (SerialBT.available() && systemAutoMode) {
    // Clear Bluetooth buffer when in auto mode to prevent interference
    while (SerialBT.available()) {
      SerialBT.read();
    }
    // Reset manual control if system switched to auto mode
    if (manualControlActive) {
      manualControlActive = false;
      Serial.println("System switched to auto mode - disabling Bluetooth override");
    }
  }

  // Check for motor control data from ESP32-CAM
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    data.trim();
    
    if (data.startsWith("M:")) {
      // This is motor control data from ESP32-CAM
      parseMotorControlData(data);
      lastActivityTime = millis(); // Reset activity timeout
    } else if (data.startsWith("MODE:")) {
      // This is mode information from ESP32-CAM
      parseModeData(data);
    }
  }

  // Read ultrasonic sensor periodically
  if (millis() - lastUltrasonicReadTime > ULTRASONIC_INTERVAL) {
    checkForObstacles();
    lastUltrasonicReadTime = millis();
  }

  // Handle obstacle avoidance if needed
  handleObstacleAvoidance();

  // Handle servo scanning when no face is detected and in face tracking mode (only in auto mode)
  if (systemAutoMode && faceTrackingMode && !faceDetected && servoContinuousMode) {
    if (millis() - lastServoMoveTime > SERVO_SCAN_DELAY) {
      scanWithServo();
      lastServoMoveTime = millis();
    }
  }

  // Check for connection timeout to stop motors
  checkTimeout();
}

// Scan with servo motor until face is detected
void scanWithServo() {
  // Move servo in steps between min and max angles
  if (servoDirectionUp) {
    servoPosition += SERVO_SCAN_STEP;
    if (servoPosition >= SERVO_MAX_ANGLE) {
      servoPosition = SERVO_MAX_ANGLE;
      servoDirectionUp = false;
    }
  } else {
    servoPosition -= SERVO_SCAN_STEP;
    if (servoPosition <= SERVO_MIN_ANGLE) {
      servoPosition = SERVO_MIN_ANGLE;
      servoDirectionUp = true;
    }
  }
  
  // Update servo position
  setServoPosition(servoPosition);
}

// Set servo position directly
void setServoPosition(int position) {
  // Constrain position to valid range
  position = constrain(position, SERVO_MIN_DUTY, SERVO_MAX_DUTY);
  // Set PWM duty cycle
  ledcWrite(SERVO_CHANNEL, position);
  servoPosition = position;
}

// Parse mode data from ESP32-CAM
void parseModeData(String data) {
  // Format: "MODE:auto_mode"
  data = data.substring(5); // Remove "MODE:"
  int mode_val = data.toInt();
  
  bool newSystemAutoMode = (mode_val == 1);
  
  if (newSystemAutoMode != systemAutoMode) {
    systemAutoMode = newSystemAutoMode;
    Serial.printf("System mode changed to: %s\n", systemAutoMode ? "AUTO" : "MANUAL");
    
    // Clear manual control when switching to auto mode
    if (systemAutoMode && manualControlActive) {
      manualControlActive = false;
      Serial.println("Switched to auto mode - disabling Bluetooth override");
    }
    
    // Update LED indicators
    if (systemAutoMode) {
      digitalWrite(LED_WIFI, HIGH);  // WiFi LED indicates auto mode
      digitalWrite(LED_BT, LOW);     // Turn off BT LED when in auto mode
    } else {
      digitalWrite(LED_WIFI, LOW);   // Turn off WiFi LED in manual mode
      // BT LED will be controlled by Bluetooth connection status
    }
  }
}

// Parse motor control data from ESP32-CAM
void parseMotorControlData(String data) {
  // Only process motor commands from ESP32-CAM when in auto mode and no manual override
  if (!systemAutoMode) {
    // In manual mode, ignore UART motor commands completely
    return;
  }
  
  if (manualControlActive) {
    Serial.println("Manual control active - ignoring UART commands");
    return;
  }

  // Format: "M:x,y,auto_mode,face_detected"
  data = data.substring(2); // Remove "M:"
  
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);
  
  if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
    float x = data.substring(0, firstComma).toFloat();
    float y = data.substring(firstComma + 1, secondComma).toFloat();
    int auto_mode = data.substring(secondComma + 1, thirdComma).toInt();
    int face_detected_val = data.substring(thirdComma + 1).toInt();
    
    // Set face tracking mode based on auto_mode flag
    faceTrackingMode = (auto_mode == 1);
    // Update face detection status
    faceDetected = (face_detected_val == 1);
    
    // Debug output
    Serial.printf("Received from ESP32-CAM: x=%.2f, y=%.2f, mode=%d, face=%d\n", 
                  x, y, auto_mode, face_detected_val);
    
    // Control motors based on received values (only in auto mode)
    if (systemAutoMode && faceTrackingMode) {
      // Use the values to control the motors
      controlMotors(x, y);
      
      // In face tracking mode with a face detected, stop servo scanning
      if (faceDetected) {
        servoContinuousMode = false;
      } else {
        // No face detected, start scanning
        servoContinuousMode = true;
      }
    } else if (systemAutoMode) {
      // Stop motors when not in face tracking mode
      stopMotors();
      // Center servo when not in face tracking mode
      setServoPosition(SERVO_CENTER);
    }
  }
}

// Handle gyroscope data received via Bluetooth
void handleBluetoothData() {
  // Only allow Bluetooth control when system is in manual mode
  if (systemAutoMode) {
    Serial.println("System in auto mode - ignoring Bluetooth commands");
    return;
  }

  String jsonData = "";
  
  // Read until we get a complete line
  while (SerialBT.available()) {
    char c = SerialBT.read();
    jsonData += c;
    
    if (c == '\n') {
      break;
    }
  }
  
  jsonData.trim();
  if (jsonData.length() == 0) return;

  // Update Bluetooth command timestamp
  lastBluetoothCommandTime = millis();

  // Determine message type: joystick or command
  if (jsonData.indexOf('x') >= 0 && jsonData.indexOf('y') >= 0 && jsonData.indexOf('cmd') < 0) {
    // Joystick data - activate manual control override (only in manual mode)
    manualControlActive = true;
    
    float jx = 0, jy = 0;
    if (sscanf(jsonData.c_str(), "{\"x\":%f,\"y\":%f}", &jx, &jy) == 2) {
      Serial.printf("Manual control active: x=%.2f, y=%.2f (system in MANUAL mode)\n", jx, jy);
      controlMotors(jx, jy);
      lastActivityTime = millis();
      
      // Visual indication of manual control
      digitalWrite(LED_BT, HIGH);
    }
  } else if (jsonData.indexOf('"cmd"') >= 0) {
    // Command data
    char cmd[16];
    bool val = false;
    if (sscanf(jsonData.c_str(), "{\"cmd\":\"%[^\"]\",\"value\":%d}", cmd, (int*)&val) >= 1) {
      String command = String(cmd);
      if (command == "power") {
        if (val && !systemAutoMode) {
          // Turn on manual control (only when system is in manual mode)
          manualControlActive = true;
          digitalWrite(LED_BT, HIGH);
          Serial.println("Manual control enabled via power command");
        } else {
          // Turn off manual control
          manualControlActive = false;
          digitalWrite(LED_BT, LOW);
          stopMotors();
          Serial.println("Manual control disabled via power command");
        }
      }
      // Note: Remove mode switching commands since mode is controlled by Flask app
    }
  }
}

// Control motors based on joystick data
void controlMotors(float x, float y) {
  // Use different speeds based on control source and movement type
  const int FULL_SPEED = 255;
  const int HALF_SPEED = 155;  // Half speed for face tracking left/right movements
  
  // Determine if this is face tracking control (when system is in auto mode and face tracking is active)
  bool isFaceTrackingControl = (systemAutoMode && faceTrackingMode && !manualControlActive);
  
  // Spin in place for pure left/right commands
  if (abs(y) < 0.1 && abs(x) > 0.1) {
    bool leftFwd = x > 0;
    bool rightFwd = !leftFwd;
    
    // Use half speed for face tracking left/right movements, full speed for manual control
    int spinSpeed = isFaceTrackingControl ? HALF_SPEED : FULL_SPEED;
    setMotors(spinSpeed, spinSpeed, leftFwd, rightFwd);
    return;
  }
  
  // Y controls forward/backward
  // X controls left/right turning
  
  // Determine if we should move based on input threshold
  bool shouldMove = (abs(y) > 0.1);  // Only move if input is significant
  
  // Forward/Backward direction
  bool goingForward = (y < 0);
  
  // If obstacle detected, prevent forward motion
  if (obstacleDetected && goingForward) {
    shouldMove = false;
  }
  
  // Calculate motor directions based on turning input
  bool leftMotorOn = false;
  bool rightMotorOn = false;
  bool leftMotorForward = goingForward;
  bool rightMotorForward = goingForward;
  
  if (!shouldMove) {
    // No movement
    leftMotorOn = false;
    rightMotorOn = false;
  } else if (abs(x) < 0.2) {
    // Going straight - both motors on at full speed
    leftMotorOn = true;
    rightMotorOn = true;
  } else if (x < -0.2) {
    // Turning left - only right motor or differential
    if (abs(x) > 0.6) {
      // Sharp left turn - only right motor
      leftMotorOn = false;
      rightMotorOn = true;
    } else {
      // Gentle left turn - both motors, left slower/stopped
      leftMotorOn = false;
      rightMotorOn = true;
    }
  } else if (x > 0.2) {
    // Turning right - only left motor or differential
    if (abs(x) > 0.6) {
      // Sharp right turn - only left motor
      leftMotorOn = true;
      rightMotorOn = false;
    } else {
      // Gentle right turn - both motors, right slower/stopped
      leftMotorOn = true;
      rightMotorOn = false;
    }
  }
  
  // Apply motor settings with appropriate speed based on movement type and control source
  int leftSpeed = 0;
  int rightSpeed = 0;
  
  if (leftMotorOn && rightMotorOn) {
    // Both motors on (straight movement) - always use full speed
    leftSpeed = FULL_SPEED;
    rightSpeed = FULL_SPEED;
  } else if (leftMotorOn || rightMotorOn) {
    // Turning movement - use half speed for face tracking, full speed for manual
    int turnSpeed = isFaceTrackingControl ? HALF_SPEED : FULL_SPEED;
    leftSpeed = leftMotorOn ? turnSpeed : 0;
    rightSpeed = rightMotorOn ? turnSpeed : 0;
  }
  
  setMotors(leftSpeed, rightSpeed, leftMotorForward, rightMotorForward);
  
  // Debug output with speed indication
  Serial.print("Motors: L=");
  Serial.print(leftSpeed);
  Serial.print(leftMotorForward ? " FWD" : " REV");
  Serial.print(" R=");
  Serial.print(rightSpeed);
  Serial.print(rightMotorForward ? " FWD" : " REV");
  if (isFaceTrackingControl && (leftSpeed == HALF_SPEED || rightSpeed == HALF_SPEED)) {
    Serial.print(" (FACE TRACKING - HALF SPEED)");
  } else if (!isFaceTrackingControl && (leftSpeed > 0 || rightSpeed > 0)) {
    Serial.print(" (MANUAL CONTROL - FULL SPEED)");
  }
  Serial.println();
}

// Set motors with specified speed and direction
void setMotors(int leftSpeed, int rightSpeed, bool leftForward, bool rightForward) {
  // Set left motor direction
  digitalWrite(MOTOR_A_IN1, leftForward ? HIGH : LOW);
  digitalWrite(MOTOR_A_IN2, leftForward ? LOW : HIGH);
  
  // Set right motor direction
  digitalWrite(MOTOR_B_IN3, rightForward ? HIGH : LOW);
  digitalWrite(MOTOR_B_IN4, rightForward ? LOW : HIGH);
  
  // Set motor speeds
  ledcWrite(PWM_A_CHANNEL, leftSpeed);
  ledcWrite(PWM_B_CHANNEL, rightSpeed);
}

// Stop both motors
void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
  
  ledcWrite(PWM_A_CHANNEL, 0);
  ledcWrite(PWM_B_CHANNEL, 0);
  
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
  
  Serial.println("Motors stopped");
}

// Measure distance using HC-SR04 ultrasonic sensor
void checkForObstacles() {
  // Clear the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the TRIG_PIN HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the ECHO_PIN, return the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30ms (about 5m)
  
  // Calculate the distance
  // Speed of sound = 343 m/s = 0.0343 cm/µs
  // Distance = (Time × Speed of sound) / 2
  distance = (duration * 0.0343) / 2;
  
  // Check if we have an obstacle
  if (distance > 0 && distance < MIN_DISTANCE) {
    if (!obstacleDetected) {
      Serial.print("Obstacle detected at ");
      Serial.print(distance);
      Serial.println(" cm");
      
      obstacleDetected = true;
      obstacleDetectedTime = millis();
      
      // Stop if moving forward
      if (leftMotorForward || rightMotorForward) {
        stopMotors();
        avoidanceManeuverActive = true;
      }
    }
  } else {
    if (obstacleDetected) {
      Serial.println("Obstacle cleared");
      obstacleDetected = false;
    }
  }
}

// Handle obstacle avoidance maneuver
void handleObstacleAvoidance() {
  if (avoidanceManeuverActive) {
    // Use full speed for avoidance maneuvers
    const int FULL_SPEED = 255;
    
    // Simple avoidance: back up slightly, then turn right
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - obstacleDetectedTime;
    
    if (elapsedTime < 500) {
      // Back up for 500ms at full speed
      setMotors(FULL_SPEED, FULL_SPEED, false, false);
    } else if (elapsedTime < 1000) {
      // Turn right for 500ms at full speed
      setMotors(FULL_SPEED, FULL_SPEED, true, false);
    } else {
      // Maneuver complete
      avoidanceManeuverActive = false;
      stopMotors();
    }
  }
}

// Check if there's been no activity and stop motors
void checkTimeout() {
  // Different timeout handling based on system mode
  if (!systemAutoMode && manualControlActive) {
    // In manual mode with active Bluetooth control, check Bluetooth timeout
    if (millis() - lastBluetoothCommandTime > BLUETOOTH_TIMEOUT_MS) {
      Serial.println("Bluetooth timeout in manual mode, stopping motors");
      stopMotors();
      manualControlActive = false;
      digitalWrite(LED_BT, LOW);
    }
  } else if (systemAutoMode) {
    // In auto mode, check UART timeout
    if (millis() - lastActivityTime > TIMEOUT_MS && btConnected && faceTrackingMode) {
      // No activity for the timeout period
      if (leftMotorSpeed > 0 || rightMotorSpeed > 0) {
        Serial.println("UART connection timeout, stopping motors");
        stopMotors();
      }
    }
  }
}

// Blink an LED n times with a specified delay
void blinkLED(int pin, int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
  }
}
