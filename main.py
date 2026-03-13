from machine import Pin, PWM, I2C
import sys
import time
import uselect

# --- HW-123 SETUP ---
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
MPU_ADDR = 0x68
i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')

# --- GYRO CALIBRATION ---
gyro_z_offset = 0.0
global_yaw = 0.0
target_heading = 0.0

for _ in range(100):
    data = i2c.readfrom_mem(MPU_ADDR, 0x47, 2)
    val = (data[0] << 8) | data[1]
    if val > 32767: val -= 65536
    gyro_z_offset += (val / 131.0); time.sleep(0.01)
gyro_z_offset /= 100.0

def get_gyro_z():
    data = i2c.readfrom_mem(MPU_ADDR, 0x47, 2)
    val = (data[0] << 8) | data[1]
    if val > 32767: val -= 65536
    return (val / 131.0) - gyro_z_offset

# --- MOTOR SETUP (Wiring Correction Integrated) ---
fl_ena = PWM(Pin(2)); fl_in1 = Pin(3, Pin.OUT); fl_in2 = Pin(4, Pin.OUT)
fr_enb = PWM(Pin(5)); fr_in3 = Pin(6, Pin.OUT); fr_in4 = Pin(7, Pin.OUT)
br_ena = PWM(Pin(18)); br_in1 = Pin(19, Pin.OUT); br_in2 = Pin(20, Pin.OUT)
bl_enb = PWM(Pin(21)); bl_in3 = Pin(22, Pin.OUT); bl_in4 = Pin(26, Pin.OUT)
for p in [fl_ena, fr_enb, bl_enb, br_ena]: p.freq(1000)

def set_motors(l, r):
    l = max(min(l, 65535), -65535); r = max(min(r, 65535), -65535)
    fl_in1.value(1 if l>0 else 0); fl_in2.value(0 if l>0 else 1)
    bl_in3.value(1 if l>0 else 0); bl_in4.value(0 if l>0 else 1)
    fr_in3.value(1 if r>0 else 0); fr_in4.value(0 if r>0 else 1)
    br_in1.value(1 if r>0 else 0); br_in2.value(0 if r>0 else 1)
    fl_ena.duty_u16(abs(int(l))); bl_enb.duty_u16(abs(int(l)))
    fr_enb.duty_u16(abs(int(r))); br_ena.duty_u16(abs(int(r)))

def turn_90(direction):
    global global_yaw, target_heading
    start_yaw = global_yaw
    timeout = time.ticks_ms()
    if direction == "LEFT": set_motors(55000, -55000)
    else: set_motors(-55000, 55000)
   
    while abs(global_yaw - start_yaw) < 88.0:
        if time.ticks_diff(time.ticks_ms(), timeout) > 3000: break
        now = time.ticks_ms() # Update global yaw inside loop
        gz = get_gyro_z()
        global_yaw += gz * 0.01
        time.sleep(0.01)
    set_motors(0, 0)
    target_heading = global_yaw

# --- PID LOOP ---
KP = 1200.0
BASE_SPEED = 25000 ##
mode = "STOP"
last_time = time.ticks_ms()

while True:
    poll = uselect.select([sys.stdin], [], [], 0)[0]
    if poll:
        line = sys.stdin.readline().strip()
        if line == "RESET_GYRO": global_yaw = 0.0; target_heading = 0.0
        elif line == "FORWARD": mode = "FORWARD"
        elif line == "STOP": mode = "STOP"; set_motors(0, 0)
        elif line == "LEFT_90": turn_90("LEFT")
        elif line == "RIGHT_90": turn_90("RIGHT")

    if mode == "FORWARD":
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_time) / 1000.0
        last_time = now
        global_yaw += get_gyro_z() * dt
        error = target_heading - global_yaw
        correction = error * KP
        set_motors(BASE_SPEED + correction, BASE_SPEED - correction)
        time.sleep(0.01)
    else:
        last_time = time.ticks_ms()
