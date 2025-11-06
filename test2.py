"""
NOTE: This code was made to run the drone via the computer, not the raspberry pi
NOTE: The COM port needs to be adjusted based on what COM it is actually connected to
NOTE: Verify baud rate is correct
NOTE: Make sure there are no other apps that are connected to the FC like Mission Planner

This code goes through the following steps
1. Connects to the FC
2. Sets flight mode to GUIDED_NOGPS
3. Arms the motors
4. Disarms
"""

from pymavlink import mavutil
import time
# -------------------------------
# USER CONFIGURATION
# -------------------------------
SERIAL_PORT = 'COM12'   # change this based on findings
BAUD_RATE = 115200

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
#mode = 'GUIDED_NOGPS'
#master.set_mode_apm(mode)
master.set_mode(20)  # Mode 20 = GUIDED_NOGPS
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
# DISARM
# -------------------------------
print("Landing complete. Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("Disarmed successfully.")