from flask import Flask, render_template, Response, request, jsonify
import cv2
import mediapipe as mp
import numpy as np
import requests
import json
import time
import threading

app = Flask(__name__)

# Initialize MediaPipe Face Detection
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.5)

# Global state
auto_mode = False  # Start in manual mode
desired_motor_x = 0.0
desired_motor_y = 0.0

# Face detection stability tracking
face_detection_history = []
STABILITY_FRAMES = 5  # Number of consecutive frames needed for stable detection
STABILITY_THRESHOLD = 0.8  # 80% of frames must detect a face to be considered stable
face_stable = False
consecutive_face_frames = 0
consecutive_no_face_frames = 0
last_stable_motor_x = 0.0
last_stable_motor_y = 0.0

# Motor control smoothing
MOTOR_SMOOTHING_FACTOR = 0.3  # Lower = more smoothing, higher = more responsive

# Face distance calculation constants
FOCAL_LENGTH_MM = 3.6  # ESP32-CAM focal length in mm
TYPICAL_FACE_HEIGHT_MM = 200  # Average human face height in mm
IMAGE_HEIGHT_PX = 240  # ESP32-CAM image height in pixels
SENSOR_HEIGHT_MM = 2.7  # ESP32-CAM sensor height in mm
# Constants for distance calculation
DISTANCE_EQ_TOP = FOCAL_LENGTH_MM * TYPICAL_FACE_HEIGHT_MM * IMAGE_HEIGHT_PX
# Smoothing factor for distance (0-1), higher = more smoothing
DISTANCE_SMOOTHING = 0.7
# Last calculated distance (for smoothing)
last_face_distance = 500  # Start at 500mm (50cm) as default

# ESP32-CAM stream URL
def get_esp32_url():
    esp32_ip = "192.168.159.39"  
    return f"http://{esp32_ip}"

def get_esp32_control_url():
    esp32_ip = "192.168.159.39"  
    return f"http://{esp32_ip}:81"

def calculate_motor_control(face_x, face_y, face_distance, frame_width, frame_height):
    """
    Calculate motor control values based on face position and distance
    Maps face position to motor control values with fixed speeds:
    - x: -1 (far left) to 1 (far right)
    - y: -1 (top) to 1 (bottom)
    - using fixed speed values for constant motor rotation
    """
    # Calculate normalized position (-1 to 1)
    # Center of frame is (0,0)
    normalized_x = (2 * face_x / frame_width) - 1
    normalized_y = (2 * face_y / frame_height) - 1
    
    # Reverse y-axis for motor control (top of frame = forward)
    normalized_y = -normalized_y
    
    # Apply deadzone in the middle (no movement when face is near center)
    deadzone = 0.2
    
    # Initialize motor control values
    motor_x = 0
    motor_y = 0
    
    # X-axis control (left/right) with fixed speeds
    if abs(normalized_x) < deadzone:
        motor_x = 0  # No movement in deadzone
    elif normalized_x < -0.6:
        motor_x = -1.0  # Full speed left
    elif normalized_x < -0.3:
        motor_x = -0.5  # Half speed left
    elif normalized_x > 0.6:
        motor_x = 1.0  # Full speed right
    elif normalized_x > 0.3:
        motor_x = 0.5  # Half speed right
    
    # Y-axis control (forward/backward) with fixed speeds
    if abs(normalized_y) < deadzone:
        motor_y = 0  # No movement in deadzone
    elif normalized_y < -0.6:
        motor_y = -1.0  # Full speed backward
    elif normalized_y < -0.3:
        motor_y = -0.5  # Half speed backward
    elif normalized_y > 0.6:
        motor_y = 1.0  # Full speed forward
    elif normalized_y > 0.3:
        motor_y = 0.5  # Half speed forward
    
    # Distance-based adjustments (using fixed thresholds)
    # Optimal distance range (in mm)
    optimal_min = 400  # 40cm
    optimal_max = 600  # 60cm
    
    if face_distance < optimal_min:
        # Too close - move backward with fixed speed
        if motor_y <= 0:  # If not already moving backward
            motor_y = 0.5  # Set to half speed backward
    elif face_distance > optimal_max:
        # Too far - move forward with fixed speed
        if motor_y >= 0:  # If not already moving forward
            motor_y = -0.5  # Set to half speed forward
    
    return motor_x, motor_y

# Track last sent control values to avoid spamming the ESP32
last_motor_x = 0
last_motor_y = 0
last_send_time = 0

def send_motor_control(motor_x, motor_y, auto_mode, face_distance=None):
    """Send motor control values to ESP32-CAM"""
    global last_motor_x, last_motor_y, last_send_time
    
    # Only send if values changed significantly or it's been a while
    current_time = time.time()
    if (abs(motor_x - last_motor_x) > 0.1 or 
        abs(motor_y - last_motor_y) > 0.1 or
        current_time - last_send_time > 0.5):  # Send at least every 0.5 seconds
        
        try:
            # Use the control server on port 81
            esp32_url = get_esp32_control_url()
            control_url = f"{esp32_url}/control"
            
            # Prepare JSON payload
            payload = {
                "x": float(motor_x),
                "y": float(motor_y),
                "auto": 1 if auto_mode else 0
            }
            
            # Add distance data if available
            if face_distance is not None:
                payload["distance"] = int(face_distance)
            
            # Send POST request to ESP32 with increased timeout
            response = requests.post(control_url, 
                                     data=json.dumps(payload),
                                     headers={"Content-Type": "application/json"},
                                     timeout=2.0)  # Increased timeout from 0.5 to 2.0 seconds
            
            print(f"Sent to ESP32: {payload}, Response: {response.status_code}")
            
            # Update last sent values
            last_motor_x = motor_x
            last_motor_y = motor_y
            last_send_time = current_time
            
        except requests.exceptions.ConnectionError as e:
            print(f"Connection error to ESP32-CAM at {esp32_url}: {e}")
            print("Check if the IP address is correct and the ESP32-CAM is online")
        except Exception as e:
            print(f"Error sending control to ESP32: {e}")

def process_frame(frame):
    # Convert the BGR image to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process the frame with MediaPipe
    results = face_detection.process(rgb_frame)
    
    # Convert back to BGR for OpenCV display
    annotated_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
    
    # Get frame dimensions
    h, w, c = annotated_frame.shape
    frame_center_x = w // 2
    frame_center_y = h // 2
    
    # Draw center lock box for reference
    box_size = 80  # Size of the center lock box
    box_half = box_size // 2
    
    # Draw the center lock box (rectangle)
    cv2.rectangle(annotated_frame, 
                  (frame_center_x - box_half, frame_center_y - box_half),
                  (frame_center_x + box_half, frame_center_y + box_half),
                  (255, 255, 255), 2)  # White border
    
    # Draw center crosshair in the lock box
    cv2.line(annotated_frame, 
             (frame_center_x - 15, frame_center_y), 
             (frame_center_x + 15, frame_center_y), 
             (255, 255, 255), 2)  # White horizontal line
    cv2.line(annotated_frame, 
             (frame_center_x, frame_center_y - 15), 
             (frame_center_x, frame_center_y + 15), 
             (255, 255, 255), 2)  # White vertical line
    
    # Add center lock box label
    cv2.putText(annotated_frame, "CENTER", 
                (frame_center_x - 30, frame_center_y - box_half - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Default motor values (no movement)
    motor_x, motor_y = 0, 0
    face_detected = False
    face_distance = None
    global last_face_distance, face_detection_history, face_stable
    global consecutive_face_frames, consecutive_no_face_frames
    global last_stable_motor_x, last_stable_motor_y
    
    # Draw face detection results
    if results.detections:
        for detection in results.detections:
            # Check detection confidence
            detection_score = detection.score[0] if detection.score else 0
            
            # Only process high-confidence detections
            if detection_score > 0.7:  # Higher confidence threshold
                mp_drawing.draw_detection(annotated_frame, detection)
                
                # Get bounding box coordinates
                bbox = detection.location_data.relative_bounding_box
                x, y = int(bbox.xmin * w), int(bbox.ymin * h)
                width, height = int(bbox.width * w), int(bbox.height * h)
                
                # Calculate face center
                face_center_x = x + width // 2
                face_center_y = y + height // 2
                
                # Calculate face distance (in mm)
                distance_eq_bottom = height * SENSOR_HEIGHT_MM
                if distance_eq_bottom > 0:  # Avoid division by zero
                    raw_distance = DISTANCE_EQ_TOP / distance_eq_bottom
                    # Apply smoothing to avoid jumpy distance readings
                    face_distance = (DISTANCE_SMOOTHING * last_face_distance + 
                                    (1 - DISTANCE_SMOOTHING) * raw_distance)
                    last_face_distance = face_distance
                
                # Draw face center point and targeting crosshair
                cv2.circle(annotated_frame, (face_center_x, face_center_y), 5, (0, 255, 0), -1)
                
                # Draw face crosshair
                cv2.line(annotated_frame, (face_center_x - 20, face_center_y), (face_center_x + 20, face_center_y), (0, 255, 0), 2)
                cv2.line(annotated_frame, (face_center_x, face_center_y - 20), (face_center_x, face_center_y + 20), (0, 255, 0), 2)
                
                # Draw line from face center to center lock box if face is stable
                if face_stable and consecutive_face_frames >= STABILITY_FRAMES:
                    cv2.line(annotated_frame, 
                             (face_center_x, face_center_y), 
                             (frame_center_x, frame_center_y), 
                             (0, 255, 255), 1)  # Yellow line connecting face to center
                    
                    # Calculate distance from face to center
                    dx = face_center_x - frame_center_x
                    dy = face_center_y - frame_center_y
                    distance_to_center = int(((dx**2 + dy**2)**0.5))
                    
                    # Show distance to center if face is outside the lock box
                    if distance_to_center > box_half:
                        cv2.putText(annotated_frame, f"Off-center: {distance_to_center}px", 
                                    (10, h - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    else:
                        cv2.putText(annotated_frame, "CENTERED", 
                                    (10, h - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                face_detected = True
                consecutive_face_frames += 1
                consecutive_no_face_frames = 0
                
                # Calculate motor control values if in auto mode
                if auto_mode and face_distance is not None:
                    raw_motor_x, raw_motor_y = calculate_motor_control(face_center_x, face_center_y, face_distance, w, h)
                    
                    # Apply smoothing to motor values
                    smoothed_motor_x = (MOTOR_SMOOTHING_FACTOR * raw_motor_x + 
                                       (1 - MOTOR_SMOOTHING_FACTOR) * last_stable_motor_x)
                    smoothed_motor_y = (MOTOR_SMOOTHING_FACTOR * raw_motor_y + 
                                       (1 - MOTOR_SMOOTHING_FACTOR) * last_stable_motor_y)
                    
                    # Check if face detection is stable
                    if consecutive_face_frames >= STABILITY_FRAMES:
                        face_stable = True
                        last_stable_motor_x = smoothed_motor_x
                        last_stable_motor_y = smoothed_motor_y
                        
                        # Store stable values for background sender
                        global desired_motor_x, desired_motor_y
                        desired_motor_x, desired_motor_y = smoothed_motor_x, smoothed_motor_y
                        
                        print(f"STABLE Face at ({face_center_x},{face_center_y}) → Dist={face_distance:.0f}mm → X={smoothed_motor_x:.2f},Y={smoothed_motor_y:.2f}")
                    else:
                        print(f"DETECTING Face ({consecutive_face_frames}/{STABILITY_FRAMES}) at ({face_center_x},{face_center_y}) → Waiting for stability...")
                else:
                    print(f"Face detected - Manual mode")
                
                # Add tracking info text
                cv2.putText(annotated_frame, f"Face: ({face_center_x}, {face_center_y})", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Add confidence score
                cv2.putText(annotated_frame, f"Confidence: {detection_score:.2f}", (10, 120), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Add distance info
                if face_distance is not None:
                    dist_text = f"Distance: {face_distance:.0f}mm ({face_distance/10:.1f}cm)"
                    cv2.putText(annotated_frame, dist_text, (10, 90), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Add stability indicator
                if auto_mode:
                    if face_stable and consecutive_face_frames >= STABILITY_FRAMES:
                        stability_text = "STABLE TRACKING"
                        stability_color = (0, 255, 0)  # Green
                        cv2.putText(annotated_frame, f"Motor: X={smoothed_motor_x:.2f}, Y={smoothed_motor_y:.2f}", (10, 60), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        stability_text = f"STABILIZING ({consecutive_face_frames}/{STABILITY_FRAMES})"
                        stability_color = (0, 255, 255)  # Yellow
                    
                    cv2.putText(annotated_frame, stability_text, (10, 150), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, stability_color, 2)
                    cv2.putText(annotated_frame, "AUTO MODE", (w - 150, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    cv2.putText(annotated_frame, "MANUAL MODE", (w - 180, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                
                # Only process the first face detected
                break
    
    if not face_detected:
        consecutive_no_face_frames += 1
        consecutive_face_frames = 0
        
        # Reset stability after losing face for a while
        if consecutive_no_face_frames >= STABILITY_FRAMES:
            face_stable = False
            
        cv2.putText(annotated_frame, "No face detected", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        if auto_mode:
            # Only reset motor values if we've lost the face for enough frames
            if consecutive_no_face_frames >= STABILITY_FRAMES:
                desired_motor_x = desired_motor_y = 0.0
                last_stable_motor_x = last_stable_motor_y = 0.0
                print("Face lost - stopping motors after stability timeout")
            else:
                print(f"Face lost ({consecutive_no_face_frames}/{STABILITY_FRAMES}) - maintaining last stable position")
        else:
            print("No face detected - Manual mode")
    
    # Update face detection history
    face_detection_history.append(1 if face_detected else 0)
    if len(face_detection_history) > STABILITY_FRAMES:
        face_detection_history.pop(0)
    
    return annotated_frame, face_distance

def generate_frames():
    # Use ESP32-CAM stream or fallback to webcam if not available
    esp32_url = get_esp32_url()
    use_webcam = False
    
    if use_webcam:
        # Fallback to webcam
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("Could not open webcam")
        
        while True:
            success, frame = cap.read()
            if not success:
                break
                
            processed_frame, _ = process_frame(frame)
            
            # Encode as JPEG
            ret, buffer = cv2.imencode('.jpg', processed_frame)
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    else:
        # Use ESP32-CAM MJPEG stream
        while True:  # Add continuous loop to reconnect if stream fails
            try:
                stream_url = f"{esp32_url}"
                print(f"Connecting to ESP32-CAM stream at {stream_url}...")
                
                # For MJPEG streams, we need to parse the multipart response
                r = requests.get(stream_url, stream=True, timeout=5.0)  # Increased timeout
                bytes_data = bytes()
                
                for chunk in r.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')  # JPEG start
                    b = bytes_data.find(b'\xff\xd9')  # JPEG end
                    
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        
                        # Decode image
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        
                        # Process frame with face detection
                        processed_frame, _ = process_frame(frame)
                        
                        # Encode as JPEG
                        ret, buffer = cv2.imencode('.jpg', processed_frame)
                        frame_bytes = buffer.tobytes()
                        
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            except Exception as e:
                print(f"Error connecting to ESP32-CAM: {e}")
                print("Retrying in 5 seconds...")
                time.sleep(5)  # Wait before retrying
                continue  # Retry connection

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control', methods=['POST'])
def control():
    global auto_mode, desired_motor_x, desired_motor_y
    data = request.get_json()
    if 'mode' in data:
        # reset outputs on mode toggle
        auto_mode = (data['mode'] == 'auto')
        desired_motor_x = desired_motor_y = 0.0
        print(f"Mode changed to: {'AUTO' if auto_mode else 'MANUAL'}")
    return jsonify(status='success', mode='auto' if auto_mode else 'manual')

def control_sender():
    """Background thread: send control every 100ms without blocking video."""
    last_x = last_y = None
    last_mode = None
    last_distance = None
    last_face_status = None
    esp32_connected = False
    connection_retry_count = 0
    consecutive_failures = 0
    
    while True:
        try:
            # Always send current mode status to ensure ESP32 knows the current mode
            current_mode = auto_mode
            
            if current_mode:  # Auto mode
                x, y = desired_motor_x, desired_motor_y
                
                # Only send commands if face detection is stable or if we need to stop
                current_face_stable = face_stable and (consecutive_face_frames >= STABILITY_FRAMES)
                face_detected = current_face_stable
                
                # Send updates when:
                # 1. Motor values changed significantly
                # 2. Mode changed
                # 3. Face stability status changed
                # 4. Connection was just established
                if (abs(x - (last_x or 0)) > 0.05 or 
                    abs(y - (last_y or 0)) > 0.05 or 
                    last_mode != current_mode or 
                    last_face_status != face_detected or 
                    not esp32_connected):
                    
                    # Prepare JSON payload for auto mode
                    payload = {
                        "x": float(x),
                        "y": float(y),
                        "auto": 1,  # Always 1 in auto mode
                        "face_detected": 1 if face_detected else 0
                    }
                    
                    # Add distance if available
                    if last_face_distance is not None:
                        payload["distance"] = int(last_face_distance)
                    
                    # Send POST request to ESP32
                    esp32_url = get_esp32_control_url()
                    control_url = f"{esp32_url}/control"
                    
                    response = requests.post(
                        control_url,
                        data=json.dumps(payload),
                        headers={"Content-Type": "application/json"},
                        timeout=0.5
                    )
                    
                    if response.status_code == 200:
                        dist_str = f", dist={int(last_face_distance)}mm" if last_face_distance is not None else ""
                        face_str = ", STABLE TRACKING" if face_detected else ", SEARCHING"
                        stability_str = f" (stable={current_face_stable})"
                        print(f"✅ AUTO MODE: x={x:.2f}, y={y:.2f}{dist_str}{face_str}{stability_str}")
                        last_x, last_y = x, y
                        last_distance = last_face_distance
                        last_face_status = face_detected
                        esp32_connected = True
                        connection_retry_count = 0
                        consecutive_failures = 0
                    else:
                        print(f"⚠️ ESP32 returned error: {response.status_code}")
                        consecutive_failures += 1
            
            else:  # Manual mode
                if last_mode != current_mode or not esp32_connected:
                    # Send manual mode command to ESP32
                    payload = {
                        "x": 0.0, 
                        "y": 0.0, 
                        "auto": 0,  # 0 for manual mode
                        "face_detected": 0
                    }
                    
                    esp32_url = get_esp32_control_url()
                    control_url = f"{esp32_url}/control"
                    
                    response = requests.post(
                        control_url,
                        data=json.dumps(payload),
                        headers={"Content-Type": "application/json"},
                        timeout=0.5
                    )
                    
                    if response.status_code == 200:
                        print("✅ MANUAL MODE: Bluetooth control enabled, face tracking disabled")
                        esp32_connected = True
                        connection_retry_count = 0
                        consecutive_failures = 0
            
            # Update mode tracking
            last_mode = current_mode
            
        except requests.exceptions.Timeout:
            consecutive_failures += 1
            if consecutive_failures < 3 or consecutive_failures % 10 == 0:
                print(f"⏱️ Timeout connecting to ESP32-CAM at {get_esp32_url()}")
            
        except requests.exceptions.ConnectionError:
            consecutive_failures += 1
            connection_retry_count += 1
            esp32_connected = False
            
            # Only log occasionally to reduce spam
            if connection_retry_count % 10 == 0:
                print(f"❌ Cannot connect to ESP32-CAM at {get_esp32_url()}")
                print(f"   Check if the IP address is correct. Retry count: {connection_retry_count}")
        
        except Exception as e:
            consecutive_failures += 1
            print(f"❌ Error in control sender: {str(e)}")
        
        # Adaptive sleep based on connection status
        if consecutive_failures == 0:
            time.sleep(0.1)  # Normal operation - 10 updates per second
        elif consecutive_failures < 5:
            time.sleep(0.2)  # Small issues - 5 updates per second
        else:
            time.sleep(1.0)  # Major connectivity problems

# start background sender before Flask runs
threading.Thread(target=control_sender, daemon=True).start()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
