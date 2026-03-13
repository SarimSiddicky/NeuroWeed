import os
import time
import cv2
import serial
import sys
from ultralytics import YOLO
from picamera2 import Picamera2
from gpiozero import PWMOutputDevice, DigitalOutputDevice

os.environ["LD_PRELOAD"] = "/usr/lib/aarch64-linux-gnu/libopenblas.so.3"

# --- PUMP SETUP ---
pump_ena = PWMOutputDevice(14)
pump_in1 = DigitalOutputDevice(6)
pump_in2 = DigitalOutputDevice(12)

def spray_now(duration=1.2):
    print(">>> [PUMP] Spraying...")
    pump_in1.on(); pump_in2.off(); pump_ena.value = 0.6
    time.sleep(duration)
    pump_ena.value = 0; pump_in1.off(); pump_in2.off()

# --- SERIAL SETUP ---
try:
    pico = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
    pico.dtr = True
    time.sleep(2)
    print(">>> Connected to Pico")
except Exception as e:
    print(f"Serial Error: {e}"); pico = None

def send(cmd):
    if pico and pico.is_open:
        pico.write((cmd + "\n").encode('utf-8'))

# --- YOLO & CAMERA ---
model = YOLO("best_ncnn_model")
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"format": "BGR888", "size": (640, 480)})
picam2.configure(config); picam2.start()

KNOWN_WIDTH = 3.0; FOCAL_LENGTH = 920  
def get_distance(px): return (KNOWN_WIDTH * FOCAL_LENGTH) / px if px > 0 else 999

# --- FIELD SETUP ---
VELOCITY_CM_S = 17.02
THRESHOLD_CM = 20.0

if not sys.stdin.isatty():
    FIELD_LENGTH = 120.0; FIELD_WIDTH = 100.0; LANE_GAP = 20
else:
    FIELD_LENGTH = 120.0; FIELD_WIDTH = 100.0; LANE_GAP = 20

def execute_u_turn(direction, gap_cm):
    print(f">>> U-Turn: {direction}")
    send(f"{direction}_90")
    time.sleep(2.5)
    gap_time = gap_cm / VELOCITY_CM_S
    send("FORWARD")
    time.sleep(gap_time)
    send("STOP")
    time.sleep(0.5)
    send(f"{direction}_90")
    time.sleep(2.5)

# --- PATROL STATE ---
is_running = False
total_distance_cm = 0.0
covered_width_cm = 0.0
turn_direction = "LEFT"
stutter_state = "STOPPED"
DRIVE_TIME = 0.3; STOP_TIME = 0.5
last_toggle_time = time.time(); last_eval_time = time.time()
last_spray_time = 0.0

try:
    while True:
        frame = picam2.capture_array()
        cv2.imshow("WeedBot Feed", frame)
        key = cv2.waitKey(1) & 0xFF
       
        if key == ord('q'):
            is_running = not is_running
            if is_running:
                send("RESET_GYRO") # Zero the heading where we are currently facing
                time.sleep(0.1)
                stutter_state = "DRIVING"; total_distance_cm = 0.0
                last_toggle_time = time.time(); last_eval_time = time.time()
                send("FORWARD")
            else:
                stutter_state = "STOPPED"; send("STOP")
        elif key == 27: break

        if is_running:
            now = time.time(); dt = now - last_eval_time; last_eval_time = now
            if stutter_state == "DRIVING":
                total_distance_cm += (VELOCITY_CM_S * dt)
                if total_distance_cm >= FIELD_LENGTH:
                    send("STOP"); stutter_state = "PAUSED"
                    execute_u_turn(turn_direction, LANE_GAP)
                    covered_width_cm += LANE_GAP
                    if covered_width_cm >= FIELD_WIDTH: is_running = False; continue
                    turn_direction = "RIGHT" if turn_direction == "LEFT" else "LEFT"
                    total_distance_cm = 0.0; stutter_state = "DRIVING"
                    last_toggle_time = now; send("FORWARD"); continue
                if now - last_toggle_time > DRIVE_TIME:
                    send("STOP"); stutter_state = "PAUSED"; last_toggle_time = now
            elif stutter_state == "PAUSED":
                if now - last_toggle_time > STOP_TIME:
                    send("FORWARD"); stutter_state = "DRIVING"
                    last_toggle_time = now; last_eval_time = time.time()

            # YOLO Check
            results = model.predict(source=frame, conf=0.5, imgsz=320, verbose=False)
            weed_near = False
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    if get_distance(x2-x1) <= THRESHOLD_CM: weed_near = True
           
            if weed_near and (now - last_spray_time > 2.0):
                send("STOP"); time.sleep(0.5); spray_now(1.2)
                last_spray_time = time.time(); stutter_state = "DRIVING"
                last_toggle_time = time.time(); send("FORWARD")
finally:
    send("STOP"); picam2.stop(); cv2.destroyAllWindows()
