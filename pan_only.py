import cv2
import mediapipe as mp
import time
import numpy as np
import threading
import serial

# ==========================================
# 1. HARDWARE CONFIGURATION
# ==========================================
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
PAN_ID = 6
TILT_ID = 5

# Camera Stream (i-PRO)
USERNAME = "admin"
PASSWORD = "Swan1234"
IP = "192.168.0.10"
RTSP_URL = f"rtsp://{USERNAME}:{PASSWORD}@{IP}/MediaInput/stream_1"

# Control Parameters
PAN_LIMIT = (0, 4094)
TILT_LIMIT = (0, 4094)
# Configurable Tilt Limits
TILT_MIN_LIMIT = 1500 
TILT_MAX_LIMIT = 3000
KP_PAN = 0.5   # Proportional Gain for Pan
KP_TILT = 0.5  # Proportional Gain for Tilt
# Deadzones
MAIN_DEADZONE = 120 # Pixel distance for MAIN center (Large)
SERVO_SPEED =100   # Steps/Sec (0=Max, 1023=Fastest controlled, 1=Slowest)

# ==========================================
# 2. SERVO CONTROLLER (Merged from testing.py & zero_set.py)
# ==========================================
class ST3215:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            print(f"✅ Connected to {port} at {baudrate} baud")
        except Exception as e:
            print(f"❌ Failed to connect to serial port: {e}")
            exit()

    def ping(self, servo_id: int) -> bool:
        length = 2
        instruction = 0x01
        checksum = (~(servo_id + length + instruction)) & 0xFF
        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction, checksum])

        self.ser.reset_input_buffer()
        self.ser.write(packet)

        buffer = bytearray()
        start_time = time.time()

        while time.time() - start_time < 0.02:
            data = self.ser.read(32)
            if data:
                buffer.extend(data)

            if len(buffer) >= 6:
                for i in range(len(buffer) - 1):
                    if buffer[i] == 0xFF and buffer[i+1] == 0xFF:
                        if len(buffer) >= i + 6:
                            packet = buffer[i:i+6]
                            recv_id = packet[2]
                            return recv_id == servo_id
        return False

    def move(self, servo_id: int, position: int, speed: int = 0):
        """Move servo to a given position (0-4094) with optional speed (0-1023)."""
        position = max(0, min(4094, int(position)))
        speed = max(0, min(1023, int(speed)))

        pos_low = position & 0xFF
        pos_high = (position >> 8) & 0xFF
        speed_low = speed & 0xFF
        speed_high = (speed >> 8) & 0xFF

        length = 7
        instruction = 0x03
        address = 0x2A 
        checksum = (~(servo_id + length + instruction + address + pos_low + pos_high + speed_low + speed_high)) & 0xFF

        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction, address, pos_low, pos_high, speed_low, speed_high, checksum])
        self.ser.write(packet)
        # self.ser.flush()

    def read_position(self, servo_id: int) -> int | None:
        """Read 12-bit current position (0-4094)"""
        length = 4
        instruction = 0x02
        address = 0x38 
        data_len = 2
        checksum = (~(servo_id + length + instruction + address + data_len)) & 0xFF

        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction, address, data_len, checksum])
        self.ser.reset_input_buffer()
        self.ser.write(packet)

        buffer = bytearray()
        start_time = time.time()

        while time.time() - start_time < 0.05:
            data = self.ser.read(32)
            if data:
                buffer.extend(data)

            for i in range(len(buffer) - 6):
                if buffer[i] == 0xFF and buffer[i+1] == 0xFF and buffer[i+2] == servo_id and buffer[i+3] == 4:
                    pos_low = buffer[i+5]
                    pos_high = buffer[i+6]
                    position = (pos_high << 8) | pos_low
                    return max(0, min(4094, position))
        return None

    def DefineMiddle(self, servo_id):
        """
        Set the current servo position as the middle (neutral) position (2048) and enable torque.
        From zero_set.py
        """
        print(f"Make ZERO for ID {servo_id}")
        middle_pos = 2048

        # Move servo to middle
        self.move(servo_id, middle_pos, speed=0) 

        # Enable torque (write 128 to torque switch register 0x28)
        torque_enable = 128
        length = 4
        instruction = 0x03 
        address = 0x28 
        low_byte = torque_enable & 0xFF
        checksum = (~(servo_id + length + instruction + address + low_byte)) & 0xFF

        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction, address, low_byte, checksum])
        self.ser.write(packet)
        
    def TorqueDisable(self, servo_id):
        """Disable torque for a specific servo ID."""
        torque_enable = 0
        length = 4
        instruction = 0x03
        address = 0x28
        low_byte = torque_enable & 0xFF
        checksum = (~(servo_id + length + instruction + address + low_byte)) & 0xFF
        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction, address, low_byte, checksum])
        self.ser.write(packet)

    def Stop(self, servo_id):
        """Stop the servo immediately at its current position."""
        current_pos = self.read_position(servo_id)
        if current_pos is not None:
             # Move to current position with Speed 0 (Max speed / Instant hold)
             self.move(servo_id, current_pos, speed=0)

    def close(self):
        self.ser.close()


class VideoCaptureThreading:
    def __init__(self, src=0):
        self.src = src
        self.cap = cv2.VideoCapture(self.src)
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

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
            time.sleep(0.005) # Slight yield

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

# ==========================================
# 4. MAIN SYSTEM
# ==========================================
def main():
    print("🚀 Starting All System Tracking...")
    
    # 1. Initialize Controller
    controller = ST3215(SERIAL_PORT, BAUDRATE)
    
    # Ping Servos
    # Ping Servos (Timeout Logic)
    timeout_start = time.time()
    pan_ok = False
    # tilt_ok = False # Disabled
    
    print("⏳ Connecting to Servos (5s Timeout)...")
    while time.time() - timeout_start < 5.0:
        if not pan_ok:
            if controller.ping(PAN_ID): pan_ok = True
        # if not tilt_ok:
        #    if controller.ping(TILT_ID): tilt_ok = True
            
        if pan_ok: # and tilt_ok:
             print("✅ Pan Servo Connected!")
             break
        time.sleep(0.5)
        
    if not pan_ok:
        print(f"❌ Error: Pan Servo {PAN_ID} not responding after 5s. Terminating.")
        return # Terminate
    # if not tilt_ok:
    #    print(f"❌ Error: Tilt Servo {TILT_ID} not responding after 5s. Terminating.")
    #    return # Terminate

    # Get Initial Positions
    curr_pan = controller.read_position(PAN_ID)
    curr_tilt = 2048 # controller.read_position(TILT_ID) # Disabled
    
    if curr_pan is None: curr_pan = 2048
    if curr_tilt is None: curr_tilt = 2048
    
    print(f"Initial Pos -> Pan: {curr_pan}, Tilt: {curr_tilt}")

    # 2. Initialize Camera
    cap = VideoCaptureThreading(RTSP_URL)
    cap.start()
    time.sleep(1) # Warmup

    # 3. Initialize MediaPipe
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=0 # Optimized for Jetson
    )

    ws, hs = 0, 0
    # Get one frame for dims
    ret, frame = cap.read()
    if ret:
        hs, ws = frame.shape[:2]
        print(f"Resolution: {ws}x{hs}")
    else:
        print("❌ Failed to get frame from camera")
        return

    # Tracking Loop Support
    frame_count = 0
    initial_centering_done = False # Interlock Flag
    
    try:
        while True:
            success, img = cap.read()
            if not success:
                print("Stream lost.")
                break
                
            frame_count += 1
            
            # Optimization: Inference on small image
            inf_w, inf_h = 480, 270
            small_frame = cv2.resize(img, (inf_w, inf_h))
            rgb = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
            
            results = pose.process(rgb)
            
            target_x, target_y = None, None
            
            # center of image
            center_x = ws // 2
            center_y = hs // 2

            should_move = False
            error_x = 0
            error_y = 0
            
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                # Calculate Human Center (Midpoint of shoulders)
                left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
                right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
                
                # Denormalize
                target_x = int((left_shoulder.x + right_shoulder.x) / 2 * ws)
                target_y = int((left_shoulder.y + right_shoulder.y) / 2 * hs)
                
                # Draw Target (Yellow Dot)
                cv2.circle(img, (target_x, target_y), 10, (0, 255, 255), -1)
                
                # Calculate Error
                error_x = center_x - target_x # If target is left (less), error is positive.
                # Standard convention: 
                # Target Left -> Need to Pan Left (Decrease Angle? Depends on mounting)
                # Assuming Pan Servo: 0=Right, 4096=Left? OR 0=Left, 4096=Right?
                # Usually: 
                # If target_x < center_x (Target is LEFT):
                # We need to turn camera LEFT.
                # Let's assume standard servo: 2048 is center.
                # If we increase angle, does it turn Left or Right?
                # We will use the existing polarity from ptz_cam.py (KP_PAN = 0.1).
                # ptz_cam.py: current_pan += error_x * KP_PAN
                # if error_x > 0 (Target LEFT), Pan increases.
                
                error_y = center_y - target_y
            else:
                # Target Lost / Out of Range
                cv2.putText(img, "TARGET LOST - TORQUE OFF", (ws - 400, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                controller.TorqueDisable(PAN_ID)
                controller.TorqueDisable(TILT_ID)
                target_x = None # Ensure it is None so we don't process tracking logic
                
            # Check if inside Main Center Deadzone
            in_main_deadzone = (abs(error_x) < MAIN_DEADZONE) and (abs(error_y) < MAIN_DEADZONE)
            
            # STARTUP INTERLOCK
            if not initial_centering_done:
                if in_main_deadzone:
                    initial_centering_done = True
                    print("✅ TARGET LOCKED IN CENTER. ENGAGING SERVOS.")
                else:
                    # Waiting for target
                    cv2.putText(img, "WAITING FOR TARGET CENTER...", (ws - 500, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    pass # Do NOT move servos yet

            if initial_centering_done:
                # CONDITION: Stop if in Main Center
                if in_main_deadzone:
                    # CENTERED/STOPPED
                    cv2.putText(img, "TARGET REACHED - STOPPING", (ws - 400, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # IMMEDIATE STOP
                    controller.Stop(PAN_ID)
                    
                    # Trigger Zero Set logic
                    controller.DefineMiddle(PAN_ID)
                    controller.DefineMiddle(TILT_ID)
                    
                    # Reset current tracker variables to neutral 2048 effectively
                    curr_pan = 2048
                    curr_tilt = 2048
                    
                else:
                    # MOVE Logic (Only if NOT in a deadzone)
                    
                    # Update Pan (Step Logic: +/- 45 degrees)
                    # 45 degrees ~ 512 steps (assuming 4096 = 360 deg)
                    STEP_45_DEG = 512
                    
                    if not in_main_deadzone:
                        if error_x > 0: 
                            # Target is LEFT of center -> Rotate Counter-Clockwise (Decrease)
                            curr_pan -= STEP_45_DEG
                        elif error_x < 0:
                            # Target is RIGHT of center -> Rotate Clockwise (Increase)
                            curr_pan += STEP_45_DEG
                        
                    # Clamp
                    curr_pan = max(0, min(4094, curr_pan))
                    # curr_tilt = max(TILT_MIN_LIMIT, min(TILT_MAX_LIMIT, curr_tilt))
                    
                    # Send Command
                    controller.move(PAN_ID, curr_pan, SERVO_SPEED)
                    # controller.move(TILT_ID, curr_tilt) # Tilt Disabled
        
            # UI: Draw Grids/Zones
            # 1. Main Center Deadzone
            cv2.rectangle(img, (center_x - MAIN_DEADZONE, center_y - MAIN_DEADZONE), 
                               (center_x + MAIN_DEADZONE, center_y + MAIN_DEADZONE), (0, 0, 255), 2)
            
            # Grid lines
            cv2.line(img, (center_x, 0), (center_x, hs), (255, 0, 0), 1)
            cv2.line(img, (0, center_y), (ws, center_y), (255, 0, 0), 1)

            # Display Status (Right Side)
            status = f"Pan: {int(curr_pan)} Tilt: {int(curr_tilt)}"
            cv2.putText(img, status, (ws - 350, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Show
            disp = cv2.resize(img, (960, 540))
            cv2.imshow("All System", disp)
            
            # Handle Keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        controller.close()
        cv2.destroyAllWindows()
        print("System Stopped")

if __name__ == "__main__":
    main()
