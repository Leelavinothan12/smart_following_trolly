from flask import Flask, render_template, Response
import numpy as np
import time
import cv2
from threading import Thread, Lock
from picamera2 import Picamera2
from tflite_runtime.interpreter import Interpreter
import RPi.GPIO as GPIO
import os

# Initialize Flask app
app = Flask(_name_)

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor pins
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

# Ultrasonic sensor pins
TRIG = 23
ECHO = 24

# Setup GPIO pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Tracking parameters
threshold = 0.5  # Confidence threshold
tolerance = 0.1   # Center tolerance
x_deviation = 0
y_deviation = 0
arr_track_data = [0, 0, 0, 0, 0, 0]  # Tracking data
obstacle_distance = 100  # Initial distance
sensor_lock = Lock()  # For thread-safe distance reading
stop_flag = False  # Emergency stop flag
frame_skip = 3  # Process every 3rd frame

# Motor control functions
def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def backward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

# Ultrasonic sensor functions
def get_distance():
    global obstacle_distance
    while not stop_flag:
        # Send pulse
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        
        pulse_start = time.time()
        pulse_end = time.time()
        
        # Wait for echo start
        timeout = time.time() + 0.04  # 40ms timeout (~7m range)
        while GPIO.input(ECHO) == 0 and time.time() < timeout:
            pulse_start = time.time()
        
        # Wait for echo end
        timeout = time.time() + 0.04
        while GPIO.input(ECHO) == 1 and time.time() < timeout:
            pulse_end = time.time()
        
        # Calculate distance in cm
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        
        with sensor_lock:
            obstacle_distance = distance if 2 < distance < 400 else obstacle_distance
        
        time.sleep(0.1)  # 10Hz update rate

# Load YOLOv5 Nano model
def load_model(model_path):
    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    return interpreter, input_details, output_details

# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (320, 240), "format": "RGB888"})  # Lower resolution
picam2.configure(config)
picam2.start()
time.sleep(2)  # Let camera settle

# Process detections and track object
def track_object(detections, frame_width, frame_height):
    global x_deviation, y_deviation, arr_track_data
    if len(detections) == 0:
        print("No objects detected")
        stop()
        arr_track_data = [0, 0, 0, 0, "No object", 0]
        return
    
    # Find the detection with highest confidence
    best_det = max(detections, key=lambda x: x[4])
    
    x, y, w, h, conf, cls_id = best_det
    
    # Calculate center coordinates (normalized 0-1)
    x_center = x
    y_center = y
    
    # Calculate deviations from center
    x_deviation = round(0.5 - x_center, 3)
    y_deviation = round(0.5 - y_center, 3)
    
    print(f"Object center: ({x_center}, {y_center}), Deviation: ({x_deviation}, {y_deviation})")
    
    # Start robot movement in a separate thread
    thread = Thread(target=move_robot_with_obstacle_check)
    thread.start()
    
    # Update tracking data
    arr_track_data[0] = x_center
    arr_track_data[1] = y_center
    arr_track_data[2] = x_deviation
    arr_track_data[3] = y_deviation

# Move robot with obstacle checking
def move_robot_with_obstacle_check():
    global x_deviation, y_deviation, tolerance, arr_track_data, obstacle_distance
    safe_distance = 30  # cm
    with sensor_lock:
        current_distance = obstacle_distance
    
    print(f"Moving robot... X dev: {x_deviation}, Y dev: {y_deviation}, Obstacle: {current_distance}cm")
    
    if current_distance < safe_distance:
        cmd = "Obstacle!"
        delay1 = 0.5
        stop()
        time.sleep(0.5)
        backward()
        time.sleep(1)
        stop()
        time.sleep(0.5)
        right()  # Turn right to avoid obstacle
        time.sleep(1)
        stop()
    elif abs(x_deviation) < tolerance and abs(y_deviation) < tolerance:
        cmd = "Stop"
        delay1 = 0
        stop()
    else:
        if abs(x_deviation) > abs(y_deviation):
            if x_deviation >= tolerance:
                cmd = "Move Left"
                delay1 = get_delay(x_deviation, 'l')
                left()
                time.sleep(delay1)
                stop()
            elif x_deviation <= -tolerance:
                cmd = "Move Right"
                delay1 = get_delay(x_deviation, 'r')
                right()
                time.sleep(delay1)
                stop()
        else:
            if y_deviation >= tolerance:
                cmd = "Move Forward"
                delay1 = get_delay(y_deviation, 'f')
                forward()
                time.sleep(delay1)
                stop()
            elif y_deviation <= -tolerance:
                cmd = "Move Backward"
                delay1 = get_delay(y_deviation, 'b')
                backward()
                time.sleep(delay1)
                stop()
    
    arr_track_data[4] = cmd
    arr_track_data[5] = delay1

# Calculate movement delay based on deviation
def get_delay(deviation, direction):
    deviation = abs(deviation)
    if direction in ['f', 'b']:
        # Slower speeds for larger deviations
        if deviation >= 0.3:
            d = 0.1  # Slower speed for larger deviation
        elif deviation >= 0.2:
            d = 0.08  # Slightly faster
        else:
            d = 0.05  # Default speed for small deviations
    else:
        # For left/right
        if deviation >= 0.4:
            d = 0.1  # Slower speed for larger deviation
        elif deviation >= 0.35:
            d = 0.08  # Slightly faster
        else:
            d = 0.05  # Default speed for small deviations
    return d

# Flask routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Main frame generation function
def gen_frames():
    global stop_flag, frame_skip_counter
    frame_skip_counter = 0
    
    # Load model
    model_path = "bestt.tflite"  # Your YOLOv5 Nano model
    interpreter, input_details, output_details = load_model(model_path)
    input_shape = input_details[0]['shape']
    input_height, input_width = input_shape[1], input_shape[2]
    
    # Start ultrasonic sensor thread
    sensor_thread = Thread(target=get_distance)
    sensor_thread.start()
    
    try:
        while True:
            start_time = time.time()
            
            # Capture frame with Picamera2
            frame = picam2.capture_array()
            
            # Skip frames if needed
            frame_skip_counter += 1
            if frame_skip_counter % frame_skip != 0:
                continue  # Skip this frame
            
            # Preprocess frame
            frame_rgb = frame  # Picamera2 already gives RGB
            frame_resized = cv2.resize(frame_rgb, (input_width, input_height))
            input_data = np.expand_dims(frame_resized.astype(np.float32) / 255.0, axis=0)
            
            # Run inference
            interpreter.set_tensor(input_details[0]['index'], input_data)
            interpreter.invoke()
            
            # Get output
            output_data = interpreter.get_tensor(output_details[0]['index'])[0]
            
            # Process detections
            detections = []
            for det in output_data:
                x, y, w, h, conf, cls_id, _ = det
                if conf > threshold:
                    detections.append((x, y, w, h, conf, cls_id))
            
            # Track object
            track_object(detections, frame.shape[1], frame.shape[0])
            
            # Draw overlays
            frame = draw_overlays(frame, detections, arr_track_data)
            
            # Add obstacle distance to frame
            with sensor_lock:
                dist_display = obstacle_distance
            cv2.putText(frame, f"Distance: {dist_display:.1f}cm", 
                        (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                        (0, 255, 0), 2)
            
            # Encode and yield frame to Flask
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            frame_data = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n\r\n')
    
    except Exception as e:
        print(f"Error in video feed: {e}")
        stop_flag = True
        stop()

# Draw overlays on frame
def draw_overlays(frame, detections, arr_track_data):
    for detection in detections:
        x, y, w, h, conf, cls_id = detection
        cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
    # Add tracking information as text
    cv2.putText(frame, str(arr_track_data[4]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame

if _name_ == "_main_":
    app.run(host='0.0.0.0', port=2204, threaded=True)