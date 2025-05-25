import cv2
import numpy as np
import urllib.request
from ultralytics import YOLO
import time

# ESP32-CAM URLs
stream_url = 'http://192.168.4.1/cam-hi.jpg'
control_url = 'http://192.168.4.1/control'

# Load YOLOv8 model
model = YOLO('best.pt')
conf_threshold = 0.45

# Frame dimensions
FRAME_WIDTH = 800
FRAME_HEIGHT = 600
CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2

# Tracking parameters
SERVO_SPEED = 8
last_pan_tilt = [90, 90]

# Detection tracking
DETECTION_FRAMES_THRESHOLD = 5  # Number of consecutive frames with detection
detection_counter = 0
tracking_locked = False
TRACKING_LOCK_THRESHOLD = 3  # Distance from center to consider locked

# Alert system
alert_active = False
alert_start_time = 0
ALERT_DURATION = 5  # seconds

def calculate_servo_adjustment(obj_center):
    obj_x, obj_y = obj_center
    x_offset = (obj_x - CENTER_X) / CENTER_X
    y_offset = (obj_y - CENTER_Y) / CENTER_Y
    
    new_pan = last_pan_tilt[0] - (x_offset * SERVO_SPEED)
    new_tilt = last_pan_tilt[1] + (y_offset * (SERVO_SPEED-5))
    
    return (
        max(0, min(180, int(new_pan))),
        max(0, min(180, int(new_tilt)))
    )

def send_control_command(pan=None, tilt=None, alert=False, led=False, cancel_alert=False):
    params = []
    if pan is not None and tilt is not None:
        params.append(f"pan={pan}")
        params.append(f"tilt={tilt}")
    if alert:
        params.append("alert=1")
    if led:
        params.append("led=1")
    if cancel_alert:
        params.append("cancel_alert=1")
    
    if params:
        try:
            url = f"{control_url}?{'&'.join(params)}"
            urllib.request.urlopen(url)
            if pan is not None:
                last_pan_tilt[0] = pan
            if tilt is not None:
                last_pan_tilt[1] = tilt
        except Exception as e:
            print(f"Control error: {e}")

def process_frame(frame):
    global alert_active, alert_start_time, detection_counter, tracking_locked
    
    results = model(frame, verbose=False)
    target_detected = False
    target_center = None
    
    # Process detections
    for result in results:
        for box in result.boxes:
            if float(box.conf[0]) < conf_threshold:
                continue
                
            label = model.names[int(box.cls[0])]
            if label in ['badger', 'leopard']:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                target_center = ((x1 + x2) // 2, (y1 + y2) // 2)
                target_detected = True
                
                # Visual feedback
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, target_center, 5, (0, 0, 255), -1)
                break
    
    # Update detection counter
    if target_detected:
        detection_counter = min(DETECTION_FRAMES_THRESHOLD, detection_counter + 1)
        
        # Check if tracking is locked (target near center)
        if target_center:
            distance_to_center = ((target_center[0] - CENTER_X)**2 + (target_center[1] - CENTER_Y)**2)**0.5
            tracking_locked = distance_to_center < TRACKING_LOCK_THRESHOLD
    else:
        detection_counter = max(0, detection_counter - 1)
        tracking_locked = False
    
    # LED control (on when any detection)
    send_control_command(led=target_detected)
    
    # Servo tracking
    if target_detected and target_center:
        pan, tilt = calculate_servo_adjustment(target_center)
        send_control_command(pan, tilt)
    
    # Alert logic
    if detection_counter >= DETECTION_FRAMES_THRESHOLD and tracking_locked and not alert_active:
        alert_active = True
        alert_start_time = time.time()
        send_control_command(alert=True)
    elif (not tracking_locked or detection_counter < DETECTION_FRAMES_THRESHOLD) and alert_active:
        alert_active = False
        send_control_command(cancel_alert=True)
    
    # Visual feedback
    cv2.circle(frame, (CENTER_X, CENTER_Y), 5, (255, 0, 0), -1)
    status_text = f"Alert: {'ON' if alert_active else 'OFF'}"
    tracking_text = f"Tracking: {'LOCKED' if tracking_locked else 'SEARCHING'}"
    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(frame, tracking_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    return frame

def main():
    while True:
        try:
            img_resp = urllib.request.urlopen(stream_url)
            imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
            frame = cv2.imdecode(imgnp, -1)
            
            processed_frame = process_frame(frame)
            cv2.imshow('Tracking System', processed_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
