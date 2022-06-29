import sys
import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Choose a mode
mode = 'MANUAL'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)
   
# Get mode ID
mode_id = master.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

while True:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

""""
def set_rc_channel_pwm(id, pwm=1500):
    if id < 1:
        print("Channel does not exist.")
        return

    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm
        master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_channel_values)
"""
# Arm
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

time.sleep(1)

while True:

    time.sleep(0.2)
    master.mav.manual_control_send(
        master.target_system,
        500,
        0,
        200,
        0,
        0)
    print("0")
    time.sleep(5)


    master.mav.manual_control_send(
        master.target_system,
        500,
        0,
        400,
        0,
        0)
    print("0")
    time.sleep(25)

    master.mav.manual_control_send(
        master.target_system,
        500,
        300,
        500,
        0,
        0)
    print("0")
    time.sleep(1)

    master.mav.manual_control_send(
        master.target_system,
        500,
        0,
        500,
        0,
        0)
    print("0")
    time.sleep(10)

    master.mav.manual_control_send(
        master.target_system,
        500,
        0,
        700,
        0,
        0)
    print("0")
    time.sleep(3)
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    break
