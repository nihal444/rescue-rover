/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"
#include <ArduinoJson.h>

//Replace with your network credentials
const char* ssid = "Nihal";
const char* password = "15092000";

#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// Use separate HTTP servers for streaming and control
httpd_handle_t stream_httpd = NULL;
httpd_handle_t control_httpd = NULL;

// Mutexes for thread-safe access to shared resources
SemaphoreHandle_t motorValuesMutex;

// Queue for motor control commands
QueueHandle_t motorCommandQueue;
#define QUEUE_SIZE 10
#define QUEUE_ITEM_SIZE sizeof(MotorCommand)

// Define UART pins for communication with ESP32-DEV
#define TXD2 1    // GPIO 1 (TX) of ESP32-CAM connected to RXD1 (GPIO 16) of ESP32-DEV

// Structure for motor commands
typedef struct {
  float x;
  float y;
  bool auto_mode;
  bool face_detected;  // Add face detection status
} MotorCommand;

// Global variables for motor control values
float motor_x = 0;
float motor_y = 0;
bool auto_mode = false;
bool face_detected = false;  // Add face detection status tracking
unsigned long lastTransmitTime = 0;
unsigned long lastReceivedTime = 0;  // Track when we last received a command

// Debug LED on GPIO 33 (if available)
#define DEBUG_LED 33
bool ledState = false;

// Task handles
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t commandProcessTaskHandle = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  // Add CORS headers to allow cross-origin requests
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

  while(true){
    // Toggle debug LED to indicate stream activity
    if (ledState) {
      digitalWrite(DEBUG_LED, LOW);
      ledState = false;
    } else {
      digitalWrite(DEBUG_LED, HIGH);
      ledState = true;
    }
    
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    
    // Yield for a short time to allow other tasks to run
    vTaskDelay(1);
  }
  return res;
}

// HTTP OPTIONS handler for CORS preflight requests
esp_err_t options_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
  httpd_resp_set_hdr(req, "Access-Control-Max-Age", "86400"); // 24 hours
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

// Control handler for receiving data from Python app
static esp_err_t control_handler(httpd_req_t *req) {
  // Toggle debug LED on each control request
  digitalWrite(DEBUG_LED, HIGH);
  
  char buf[100];
  int ret, remaining = req->content_len;
  
  // Check content length to prevent buffer overflow
  if (remaining > sizeof(buf) - 1) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Content too long");
    digitalWrite(DEBUG_LED, LOW);
    return ESP_FAIL;
  }
  
  // Read request content
  if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      httpd_resp_send_408(req);
    }
    digitalWrite(DEBUG_LED, LOW);
    return ESP_FAIL;
  }
  buf[ret] = '\0'; // Ensure null termination
  
  // Parse JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, buf);
  
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    digitalWrite(DEBUG_LED, LOW);
    return ESP_FAIL;
  }
  
  // Extract motor control values
  if (doc.containsKey("x") && doc.containsKey("y")) {
    MotorCommand cmd;
    cmd.x = doc["x"].as<float>();
    cmd.y = doc["y"].as<float>();
    cmd.auto_mode = doc.containsKey("auto") ? (doc["auto"].as<int>() == 1) : false;
    cmd.face_detected = doc.containsKey("face_detected") ? (doc["face_detected"].as<int>() == 1) : false;
    
    // Add command to queue instead of directly updating values
    if (xQueueSend(motorCommandQueue, &cmd, 0) == pdTRUE) {
      Serial.printf("Queued command: x=%.2f, y=%.2f, auto=%d, face=%d\n", 
                  cmd.x, cmd.y, cmd.auto_mode ? 1 : 0, cmd.face_detected ? 1 : 0);
    } else {
      Serial.println("Failed to queue command - queue full");
    }
    
    lastReceivedTime = millis(); // Update timestamp
  }
  
  // Send response with CORS headers
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
  httpd_resp_sendstr(req, "{\"status\":\"success\"}");
  
  digitalWrite(DEBUG_LED, LOW);
  return ESP_OK;
}

// Task to process incoming motor commands from the queue
void commandProcessTask(void * parameter) {
  MotorCommand cmd;
  
  while (true) {
    // Wait for a command from the queue
    if (xQueueReceive(motorCommandQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Take mutex to safely update motor values
      if (xSemaphoreTake(motorValuesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        motor_x = cmd.x;
        motor_y = cmd.y;
        auto_mode = cmd.auto_mode;
        face_detected = cmd.face_detected;
        xSemaphoreGive(motorValuesMutex);
        
        Serial.printf("Processing command: x=%.2f, y=%.2f, auto=%d, face=%d\n", 
                      cmd.x, cmd.y, cmd.auto_mode ? 1 : 0, cmd.face_detected ? 1 : 0);
      }
    }
    
    // Short delay to prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task to handle sending motor control values to ESP32-DEV
void motorControlTask(void * parameter) {
  float current_x = 0;
  float current_y = 0;
  bool current_auto = false;
  bool current_face_detected = false;
  
  while (true) {
    // Get latest motor values
    if (xSemaphoreTake(motorValuesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      current_x = motor_x;
      current_y = motor_y;
      current_auto = auto_mode;
      current_face_detected = face_detected;
      xSemaphoreGive(motorValuesMutex);
      
      // Send motor control values to ESP32-DEV with mode and face detection status
      // Format: M:x,y,auto_mode,face_detected
      Serial2.printf("M:%.2f,%.2f,%d,%d\n", 
                    current_x, current_y, 
                    current_auto ? 1 : 0,
                    current_face_detected ? 1 : 0);
                    
      // Also send mode-only updates to ensure ESP32-DEV knows current mode
      // Format: MODE:auto_mode
      Serial2.printf("MODE:%d\n", current_auto ? 1 : 0);
    }
    
    // Send every 50ms (20 times per second)
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void startCameraServer(){
  // Create mutex for thread-safe access to motor values
  motorValuesMutex = xSemaphoreCreateMutex();
  
  // Create queue for motor commands
  motorCommandQueue = xQueueCreate(QUEUE_SIZE, QUEUE_ITEM_SIZE);
  
  // Create and start command processing task (higher priority)
  xTaskCreate(
    commandProcessTask,       // Task function
    "CommandProcessTask",     // Task name
    4096,                     // Stack size (bytes)
    NULL,                     // Parameter
    2,                        // Task priority (higher than motor control)
    &commandProcessTaskHandle // Task handle
  );
  
  // Create and start motor control task
  xTaskCreate(
    motorControlTask,           // Task function
    "MotorControlTask",         // Task name
    4096,                       // Stack size (bytes)
    NULL,                       // Parameter
    1,                          // Task priority
    &motorControlTaskHandle     // Task handle
  );

  // Configure video streaming server (lower priority)
  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 80;
  stream_config.ctrl_port = 32000;  // Use a different control port
  stream_config.stack_size = 8192;  // Increase stack size
  stream_config.task_priority = 3;  // Lower priority for streaming
  stream_config.max_uri_handlers = 2;  // Just enough for stream and options
  stream_config.recv_wait_timeout = 5;  // Reduced wait timeout
  stream_config.send_wait_timeout = 5;  // Reduced send timeout
  
  // Configure control server (higher priority)
  httpd_config_t control_config = HTTPD_DEFAULT_CONFIG();
  control_config.server_port = 81;  // Different port for control
  control_config.ctrl_port = 32001; // Different control port
  control_config.stack_size = 4096; // Stack size
  control_config.task_priority = 5; // Higher priority for control
  control_config.max_uri_handlers = 2; // Just enough for control and options
  control_config.recv_wait_timeout = 3; // Quick timeout
  control_config.send_wait_timeout = 3; // Quick timeout
  
  // Start stream server
  Serial.printf("Starting stream server on port: '%d'\n", stream_config.server_port);
  if (httpd_start(&stream_httpd, &stream_config) == ESP_OK) {
    httpd_uri_t stream_uri = {
      .uri       = "/",
      .method    = HTTP_GET,
      .handler   = stream_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    
    // Add OPTIONS handler for CORS preflight requests
    httpd_uri_t stream_options_uri = {
      .uri       = "/*",
      .method    = HTTP_OPTIONS,
      .handler   = options_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(stream_httpd, &stream_options_uri);
    
    Serial.println("Stream server started");
  }
  
  // Start control server on separate port
  Serial.printf("Starting control server on port: '%d'\n", control_config.server_port);
  if (httpd_start(&control_httpd, &control_config) == ESP_OK) {
    // Add control endpoint for receiving motor commands
    httpd_uri_t control_uri = {
      .uri       = "/control",
      .method    = HTTP_POST,
      .handler   = control_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(control_httpd, &control_uri);
    
    // Add OPTIONS handler for CORS preflight requests
    httpd_uri_t control_options_uri = {
      .uri       = "/*",
      .method    = HTTP_OPTIONS,
      .handler   = options_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(control_httpd, &control_options_uri);
    
    Serial.println("Control server started");
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  
  // Initialize debug LED
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  
  // Initialize UART communication with ESP32-DEV
  Serial2.begin(115200, SERIAL_8N1, -1, TXD2); // RX pin not used, TX is GPIO 1
  Serial.println("UART connection to ESP32-DEV initialized");
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA; // VGA (640x480)
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA; // QVGA (320x240)
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  // Optimize WiFi for better performance
  WiFi.begin(ssid, password);
  WiFi.setSleep(false); // Disable WiFi sleep mode for better response
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Set to maximum power
  
  // Flash LED while connecting to WiFi
  bool led_toggle = false;
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(DEBUG_LED, led_toggle);
    led_toggle = !led_toggle;
    delay(300);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  // Show IP address
  Serial.print("Camera Stream Ready at http://");
  Serial.println(WiFi.localIP());
  Serial.print("Control API available at http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/control");
  
  // Start streaming web server
  startCameraServer();
  
  // Blink LED rapidly to indicate successful start
  for (int i = 0; i < 5; i++) {
    digitalWrite(DEBUG_LED, HIGH);
    delay(100);
    digitalWrite(DEBUG_LED, LOW);
    delay(100);
  }
}

void loop() {
  // Main loop is kept minimal since tasks handle the work
  // Just monitor WiFi connection and other system status
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost! Reconnecting...");
    
    // Visual indication of WiFi disconnection
    digitalWrite(DEBUG_LED, HIGH);
    
    WiFi.reconnect();
    
    // Wait for reconnection
    while (WiFi.status() != WL_CONNECTED) {
      digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED)); // Toggle LED
      delay(300);
      Serial.print(".");
    }
    
    digitalWrite(DEBUG_LED, LOW); // Turn off LED after reconnected
    
    Serial.println("\nWiFi reconnected");
    Serial.print("New IP: ");
    Serial.println(WiFi.localIP());
  }
  
  // Watchdog for tasks - ensure they're still running
  if (!eTaskGetState(motorControlTaskHandle) == eRunning) {
    Serial.println("⚠️ Motor control task not running! Restarting...");
    vTaskDelete(motorControlTaskHandle);
    xTaskCreate(
      motorControlTask,
      "MotorControlTask",
      4096,
      NULL,
      1,
      &motorControlTaskHandle
    );
  }
  
  // Command timeout checking - if no commands received for 5 seconds, stop the motors
  unsigned long currentTime = millis();
  if (currentTime - lastReceivedTime > 5000 && auto_mode) {
    // No commands for 5 seconds in auto mode - stop motors
    if (xSemaphoreTake(motorValuesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (motor_x != 0 || motor_y != 0) {
        Serial.println("Command timeout - stopping motors");
        motor_x = 0;
        motor_y = 0;
        // Send stop command immediately
        Serial2.printf("M:0.00,0.00,%d\n", auto_mode ? 1 : 0);
      }
      xSemaphoreGive(motorValuesMutex);
    }
  }
  
  delay(500); // Check system status every 0.5 seconds
}