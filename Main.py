"""main.py
==========================================================================================
Initialises viewer object and retrieves required control values dependent on flight phase.

Gordon Su 15/04/2020."""

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.975:
            print "Reached target altitude"
            break
        time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 10 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

def condition_yaw(heading, relative=True):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

print "Start simulator (SITL)"
import dronekit_sitl
sitl = dronekit_sitl.start_default()

# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import pandas as pd

# Initialize Variables 
N_old = 0
E_old = 0
D_old = 0 
i = 0

# Load movement commands from .csv file
commands = pd.read_csv('Commands.csv')

commands.columns


# Connect to the Vehicle.
print("Connecting to vehicle on: 127.0.0.1:14550")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Set mode to Guided, Arm and takeoff to 5 Meters
arm_and_takeoff(1.5)

print "Local Location: %s" % vehicle.location.local_frame    #NED   

time.sleep(1)


while (i < 100):
    #read command from .csv file. can instead read commands from vision code directly.
    N = commands-588['North'][i] #use 588mm from the pier as a refrence
    E = commands['East'][i]*0.001328 #convert from pixel to meters
    D = commands['Down'][i] *0.001358
    i += 1
    # Identify as Deployment Phase when no movement commands for two cycles
     
    if all( [N == 0 , N_old == 0 , E == 0 , E_old == 0 , D == 0 , D_old == 0]):
        print "Deployment position reached. Entering Deployment Phase"
        
    else :

        # Identify if there is an overshoot and apply smaller gain 
        if any( [N>0 and N_old<0, N<0 and N_old >0, E>0 and E_old<0, D>0 and D_old<0, D<0 and D_old>0]  ): 
            print "overshoot occur, applying fine adjustment"
            North = N*0.5
            East = E*0.5
            Down = D*0.5
        else:
            North = N*1
            East = E*1
            Down = D*1
            # When the code detects the drone is approaching the deployment posistion,
            # Smaller gain is used to damp the system and prevent overshoot.
            if any ([0.1>N>0 , 0>N>-0.1, 0.1>E>0 , 0>E>-0.1, 0.1>D>0 , 0>D>-0.1]):
                if 0.1>N>0 or 0>N>-0.1:
                    North = N*0.75
                    print "minor adjustment on N axis"
                if 0.1>E>0 or 0>E>-0.1:
                    East = E*0.75
                    print "minor adjustment on E axis"
                if 0.1>D>0 or 0>D>-0.1:
                    Down = D*0.75
                    print "minor adjustment on D axis"
            else :
                print "maneuvoure to deployment position"
    #Records current values of N, E and D to determine if there is overshoot in next cycle.
    N_old = N
    E_old = E
    D_old = D

    Duration= 1 #Determine how long each commands will last. actual duration = Duration/10Hz
    send_ned_velocity(North,East,Down,Duration) #sends movement command to arducopter via mavlink
    print "Local Location: %s" % vehicle.location.local_frame    #NED  
    print "N axis spped: %s m/s" %North 
    print "E axis spped: %s m/s" %East 
    print "D axis spped: %s m/s" %Down 
        
print "Landing Vehicle..."
vehicle.mode    = VehicleMode("RTL") #Return to Launch area
time.sleep (2)

print "Disconnecting from Vehicle..."
vehicle.close()
time.sleep (1)

print "Shutting Down Simulator..."
sitl.stop()
time.sleep(1)
