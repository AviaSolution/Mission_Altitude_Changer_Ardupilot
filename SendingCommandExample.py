from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

print("Waiting for heartbeat...")
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("Hearbeat received")


# Set new mode
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT, 0,
    0, 0, 0, 0, 0, 0, 200)

while True:
    # Wait for ACK command
    # Would be good to add mechanism to avoid endlessly blocking
    # if the autopilot sends a NACK or never receives the message
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Continue waiting if the acknowledged command is not `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT :
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break