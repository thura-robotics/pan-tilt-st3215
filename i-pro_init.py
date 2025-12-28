import cv2
import mediapipe as mp
import time
import numpy as np
import requests

# ==========================================
# 1. CONFIGURATION
# ==========================================
# Camera Credentials
USERNAME = "admin"
PASSWORD = "Swan1234"
IP = "192.168.0.10"
# RTSP URL (Standard)
RTSP_URL = f"rtsp://{USERNAME}:{PASSWORD}@{IP}/MediaInput/stream_1"

# Control Settings
PAN_LIMIT = (0, 180) # Keeping these for logic, but will map to camera values
TILT_LIMIT = (0, 180)
KP_PAN = 0.1   # Increased gain slightly for network delay compensation
KP_TILT = 0.1 
DEADZONE = 30

# ==========================================
# 2. SETUP (i-PRO CAMERA)
# ==========================================

print("🚀 Starting i-PRO PTZ Tracking (Jetson Optimized)...")

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
# OPTIMIZATION: Use model_complexity=0 for faster inference
pose = mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=0
)

import threading

# ... (imports)

# Threaded Video Capture to always get the LATEST frame
class VideoCaptureThreading:
    def __init__(self, src=0):
        self.src = src
        self.cap = cv2.VideoCapture(self.src)
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

    def set(self, var1, var2):
        self.cap.set(var1, var2)

    def start(self):
        if self.started:
            print('[!] Threaded video capturing has already started.')
            return None
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self.read_lock:
            frame = self.frame.copy() if self.frame is not None else None
            grabbed = self.grabbed
        return grabbed, frame

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.started = False
        self.thread.join()
        self.cap.release()

# ... (Configuration) ...

# Connect to RTSP Stream
print(f"Connecting to RTSP: {RTSP_URL}")
# Use standard FFMPEG but with threading to drop stale buffers
cap = VideoCaptureThreading(RTSP_URL)
cap.start()

# Wait a moment for the buffer to fill
time.sleep(1)

if not cap.isOpened():
    print("❌ Failed to open RTSP stream. Check connection.")
    exit()

# We need to read one frame to get dimensions
success, img = cap.read()
if success:
    hs, ws = img.shape[:2]
    print(f"Resolution: {ws}x{hs}")
else:
    ws, hs = 640, 480 # Fallback

# Initial Virtual Position (0-180 scale)
# ...

# Initial Virtual Position (0-180 scale)
current_pan = 90.0
current_tilt = 90.0

def send_camera_command(pan, tilt):
    """
    Sends Pan/Tilt commands to the i-PRO camera via CGI.
    """
    # 1. Clip values to our logical range (0-180)
    pan = np.clip(pan, PAN_LIMIT[0], PAN_LIMIT[1])
    tilt = np.clip(tilt, TILT_LIMIT[0], TILT_LIMIT[1])
    
    # 2. Map 0-180 to Camera Tracking Control
    # Note: Precise mapping depends on the specific camera model's API.
    # Standard Panasonic/i-PRO might use absolute values or simplified relative moves.
    # Here we demonstrate the request structure using the standard camctrl CGI.
    
    # Example Mapping (Hypothetical - requires calibration)
    # real_pan = map_range(pan, 0, 180, -18000, 18000)
    
    # For now, we will print the calculated logic. 
    # To actually move the camera, we would use:
    # url = f"http://{USERNAME}:{PASSWORD}@{IP}/cgi-bin/camctrl?pan={int(real_pan)}&tilt={int(real_tilt)}"
    # requests.get(url)
    
    # Since we don't have the exact degree-to-step mapping for this specific model without docs,
    # we will print the would-be command.
    # print(f"[CMD] Pan: {pan:.2f}, Tilt: {tilt:.2f}") 
    
    # --- PROPOSED IMPLEMENTATION (Uncomment if absolute control is supported) ---
    # cgi_url = f"http://{IP}/cgi-bin/camctrl?pan={int(pan*100)}&tilt={int(tilt*100)}" 
    # try:
    #     requests.get(cgi_url, auth=(USERNAME, PASSWORD), timeout=0.05) # Very short timeout
    # except Exception as e:
    #     pass # Don't block loop on network errors
    
    return pan, tilt

# ==========================================
# 3. MAIN TRACKING LOOP
# ==========================================
last_time = time.time()
frame_count = 0

while cap.isOpened():
    success, img = cap.read()
    if not success:
        print("Stream lost or empty frame.")
        # If stream is lost, try to reconnect loop or just break
        break

    frame_count += 1
    
    # Mirroring might not be desired for surveillance cameras, 
    # but good for 'mirror' style interaction. Remove flip if looking at others.
    # img = cv2.flip(img, 1) 
    
    # OPTIMIZATION: Resize just for inference (helps a lot with MediaPipe on Jetson)
    inference_h, inference_w = 270, 480 # 1/4 of 1080p roughly
    small_frame = cv2.resize(img, (inference_w, inference_h))
    rgb = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
    
    # Only process every Nth frame if really struggling, but small resolution should handle 30fps
    results = pose.process(rgb)
    
    target_detected = False
    
    if results.pose_landmarks:
        target_detected = True
        landmarks = results.pose_landmarks.landmark
        
        # Calculate Human Center (Midpoint of shoulders)
        left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        
        # Denormalize (Scale back to full resolution)
        target_x = int((left_shoulder.x + right_shoulder.x) / 2 * ws)
        target_y = int((left_shoulder.y + right_shoulder.y) / 2 * hs)

        # Draw target
        cv2.circle(img, (target_x, target_y), 10, (0, 255, 255), -1)

    # Calculate Error
        center_x = ws // 2
        center_y = hs // 2
        error_x = center_x - target_x
        error_y = center_y - target_y

        # Determine Direction Status for UI
        h_status = "CENTER"
        v_status = "CENTER"
        
        if target_x > center_x + DEADZONE:
            h_status = "RIGHT"
        elif target_x < center_x - DEADZONE:
            h_status = "LEFT"
            
        if target_y > center_y + DEADZONE:
            v_status = "DOWN"
        elif target_y < center_y - DEADZONE:
            v_status = "UP"

        # Update Virtual Angles (Control Loop)
        if abs(error_x) > DEADZONE:
            current_pan += error_x * KP_PAN
            
        if abs(error_y) > DEADZONE:
            current_tilt -= error_y * KP_TILT

    # Send Command
    real_pan, real_tilt = send_camera_command(current_pan, current_tilt)

    # UI Display
    # 1. Draw Center Grid (Grip Lines)
    cv2.line(img, (ws//2, 0), (ws//2, hs), (255, 0, 0), 1)  # Vertical Line
    cv2.line(img, (0, hs//2), (ws, hs//2), (255, 0, 0), 1)  # Horizontal Line

    # 2. Draw Deadzone Box
    cv2.rectangle(img, (ws//2 - DEADZONE, hs//2 - DEADZONE), 
                       (ws//2 + DEADZONE, hs//2 + DEADZONE), (255, 0, 0), 2)
    
    # 3. Text Info
    status_text = f"Pan: {real_pan:.1f} Tilt: {real_tilt:.1f}"
    direction_text = f"H: {h_status if 'h_status' in locals() else '---'} V: {v_status if 'v_status' in locals() else '---'}"
    
    cv2.putText(img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(img, direction_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # Resize for display if 4k
    display_img = cv2.resize(img, (960, 540))
    cv2.imshow("i-PRO Tracking", display_img)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()