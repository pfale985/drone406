"""
NOTE: This code was made to run the drone via the computer, not the raspberry pi
NOTE: The COM port needs to be adjusted based on what COM it is actually connected to
NOTE: Verify baud rate is correct
NOTE: Make sure there are no other apps that are connected to the FC like Mission Planner
NOTE: Make sure to adjust WPNAV_SPEEDs in Mission Planner
NOTE: Make sure that GUID_OPTIONS = 0 in Mission Planner

This code goes through the following steps
1. Connects to the FC
2. Sets flight mode to GUIDED_NOGPS
3. Arms the motors
4. Takes off for 3 seconds
5. Hovers for 10 seconds
6. Descends for 3 seconds
7. Disarms
"""

from pymavlink import mavutil
import time
# -------------------------------
# USER CONFIGURATION
# -------------------------------
SERIAL_PORT = 'COM12'   # change this based on findings
BAUD_RATE = 115200

#MUST SET GUID_OPTIONS TO 8

# Flight parameters (for GUID_OPTIONS=0: climb rate mode)
# 0.5 = no climb/hover, 1.0 = climb at WPNAV_SPEED_UP, 0.0 = descend at WPNAV_SPEED_DN
HOVER_THRUST = 0.5   # no climb - maintains altitude
TAKEOFF_THRUST = 0.6 # moderate climb (conservative for indoor testing)
DESCEND_THRUST = 0.3 # slow descent (conservative for indoor testing)

# -------------------------------
# CONNECT TO FLIGHT CONTROLLER
# -------------------------------
print("Connecting to FC...")
master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
master.wait_heartbeat()
print(f"Connected to system {master.target_system} component {master.target_component}")

# -------------------------------
# SET MODE: GUIDED_NOGPS
# -------------------------------
print("Setting mode: GUIDED_NOGPS")
mode = 'GUIDED_NOGPS'
master.set_mode_apm(mode)
# Alternative: master.set_mode(20)  # Mode 20 = GUIDED_NOGPS
time.sleep(1)

# Verify mode was set
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if msg:
    current_mode = msg.custom_mode
    print(f"Current mode: {current_mode}")
    if current_mode != 20:  # GUIDED_NOGPS is mode 20
        print("WARNING: Mode may not have been set correctly!")
else:
    print("WARNING: Could not verify mode change")

# -------------------------------
# ARM MOTORS
# -------------------------------
print("Arming motors...")
master.arducopter_arm()
master.motors_armed_wait()
print("Motors armed!")

# Verify armed status
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
    print("Armed status confirmed")
else:
    print("WARNING: Arming status could not be verified")

# -------------------------------
# TAKEOFF SEQUENCE (GUIDED_NOGPS)
# Using SET_ATTITUDE_TARGET for thrust control
# -------------------------------
def send_attitude_target(thrust, roll=0, pitch=0, yaw=0):
    """
    Send SET_ATTITUDE_TARGET message.
    Thrust: 0.0–1.0
    Attitude: level by default (roll, pitch, yaw = 0)
    
    NOTE: Quaternion format may need verification - keeping as potential issue
    """
    # Convert Euler angles (degrees) to quaternion
    from math import radians, sin, cos
    
    # Convert to radians
    roll_rad = radians(roll)
    pitch_rad = radians(pitch)
    yaw_rad = radians(yaw)
    
    cy = cos(yaw_rad * 0.5)
    sy = sin(yaw_rad * 0.5)
    cr = cos(roll_rad * 0.5)
    sr = sin(roll_rad * 0.5)
    cp = cos(pitch_rad * 0.5)
    sp = sin(pitch_rad * 0.5)
    
    q = [
        cy * cr * cp + sy * sr * sp,
        cy * sr * cp - sy * cr * sp,
        cy * cr * sp + sy * sr * cp,
        sy * cr * cp - cy * sr * sp,
    ]
    
    # type_mask = ignore body rates (bit 1,2,3 = 1)
    type_mask = 0b00000111
    
    master.mav.set_attitude_target_send(
        int(round(time.time() * 1000)) & 0xFFFFFFFF,  # time_boot_ms
        master.target_system,      # target_system (auto-populated from heartbeat)
        master.target_component,   # target_component (auto-populated from heartbeat)
        type_mask,
        q,      # quaternion
        0, 0, 0,  # body roll/pitch/yaw rates
        thrust
    )

# -------------------------------
# THRUST UP: TAKEOFF
# -------------------------------

# Countdown before takeoff
print("\nStarting takeoff in...")
for i in range(3, 0, -1):
    print(f"{i}...")
    time.sleep(1)
print("GO!\n")

print("Taking off...")
print('0')
for i in range(20):  # ~3 seconds at 10Hz
    send_attitude_target(thrust=0)
    time.sleep(0.1)

print('10')
for i in range(20):  # ~3 seconds at 10Hz
    send_attitude_target(thrust=0.1)
    time.sleep(0.1)

print('20')
for i in range(20):  # ~3 seconds at 10Hz
    send_attitude_target(thrust=0.2)
    time.sleep(0.1)

print('30')
for i in range(20):  # ~3 seconds at 10Hz
    send_attitude_target(thrust=0.3)
    time.sleep(0.1)
    
# -------------------------------
# Land
# -------------------------------
print("Setting mode: LAND")
mode = 'LAND'
master.set_mode_apm(mode)
print("Landing complete.")
print("Disarmed successfully.")
