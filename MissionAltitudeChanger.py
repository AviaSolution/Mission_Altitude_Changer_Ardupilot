import sys
import time
import os
import threading
import logging

from pymavlink import mavutil
from pymavlink import mavwp

altitude = 100

def DownloadWP():

    f = open(myfile, "a")
    f.write("QGC WPL 110\n")
         
    # Read Waypoint from airframe
    master.waypoint_request_list_send()
    waypoint_count = 0
            
    msg = master.recv_match(type=['MISSION_COUNT'],blocking=True)
    waypoint_count = msg.count
    print (msg.count)

    for i in range(waypoint_count):
        master.waypoint_request_send(i)
        msg = master.recv_match(type=['MISSION_ITEM'],blocking=True)
        #print ('Receving waypoint {0}'.format(msg.seq)) 
        #print (msg)
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (msg.seq,msg.current,msg.frame,msg.command,msg.param1,msg.param2,msg.param3,msg.param4,msg.x,msg.y,msg.z,msg.autocontinue)
        f.write(f"{commandline}")
 
    master.mav.mission_ack_send(master.target_system, master.target_component, 0) # OKAY

    f.close()

def ChangeMode(mode):       

        # Check if mode is available
        if mode not in master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(master.mode_mapping().keys()))
            sys.exit(1)
        #print(mode)
        
        # Get mode ID
        mode_id = master.mode_mapping()[mode]
        # Set new mode
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            217, mode_id, 0, 0, 0, 0, 0)

        while True:
            # Wait for ACK command
            # Would be good to add mechanism to avoid endlessly blocking
            # if the autopilot sends a NACK or never receives the message
            ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Continue waiting if the acknowledged command is not `set_mode`
            if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                continue

            # Print the ACK result !
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break    

def cmd_set_home(home_location, altitude):
    print('--- ', master.target_system, ',', master.target_component)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude) 

def uploadmission(alt, aFileName):
    home_location = None
    home_altitude = None

    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:   
                linearray=line.split('\t')
                ln_seq = int(linearray[0])
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=float(linearray[8])
                ln_y=float(linearray[9])
                #ln_z=float(linearray[10])
                ln_z=float(alt) # 100 m new height
                ln_autocontinue = int(float(linearray[11].strip()))
                if(i == 1):
                    home_location = (ln_x,ln_y)
                    #home_altitude = ln_z
                    # do not change home altitude because it will influce relative altitude for other WP
                    home_altitude = float(linearray[10])
                p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,
                    ln_command,
                    ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wp.add(p)
                        
    cmd_set_home(home_location,home_altitude)
    msg = master.recv_match(type = ['COMMAND_ACK'],blocking = True)
    print(msg)
    print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    time.sleep(1)

    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        master.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))
'''
# Acknowledgement from Plane
def ack (keyword)
    print("-- Message Read " + str(master.recv_match(type=keyword, blocking=True)))

# Start mission
def start_mission ()
    print("-- Mission start" )
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START, 0
        0, 0, 0, 0, 0, 0, 0)
    ack("COMMAND_ACK")

# Send message for the plane to return to the launch point
def set_return()
    print("-- Return to launch" )
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETUERN_TO_LAUNCH, 0
        0, 0, 0, 0, 0, 0, 0)
    ack("COMMAND_ACK")
'''
        
OneTime = 0
OneTimeFix = 0
WeHave= 0
                       
#----------------------------------------
myfile = "ChangedMission.txt"
# If file exists, delete it.
if os.path.isfile(myfile):
    os.remove(myfile)
else:
    # If it fails, inform the user.
    print("Info: %s file not found. A new one will be created if GPS dissapers." % myfile)

# Downloaded Mission waypoints list array
missionlist = []
#----------------------------------------
        
# Load waypoint loader
wp = mavwp.MAVWPLoader()
        
# Create the connection
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

print("Waiting for heartbeat...")
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("Hearbeat received")

while True:
            
    try:
        # If Armed then already Mission WP are uploaded, so we take them for later changes if needed
        if (OneTime == 0):
            OneTime = 1
            WeHave = 1                
            master.motors_armed_wait()
            print("Armed")
            DownloadWP()

        # After we download mission WP in to the file we need to check if GPS is fixed or not
        if (WeHave == 1):
         
            recv_messages = [
                'GPS_RAW_INT'
            ]
                            
            msg = master.recv_match(type=recv_messages, blocking=True, timeout=0.05)
            if not msg:
                continue
                        
            if msg.get_type() == 'GPS_RAW_INT':
                fix = msg.fix_type
                print("Fix Type: %s" % msg.fix_type)
        
        if (fix < 2):
            if (OneTimeFix == 0): 
                OneTimeFix = 1
                #ChangeMode("GUIDED")
                uploadmission(altitude,'ChangedMission.txt')    
                time.sleep(1)
                ChangeMode("AUTO")
                #time.sleep(1)
                #master.start_mission()
                #mission.set_current_mission_item: # by setting WP it start from it. if 0 it starts from begining        
        
                
    except KeyboardInterrupt:
        break