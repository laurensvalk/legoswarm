#!/usr/bin/env python3

from camera_client import CameraUDP
import numpy as np
import time
import logging
from hardware.motors import Motor, DriveBase, Picker
from hardware.sensors import BallSensor, Battery
from springs import Spring

#################################################################
###### Init
#################################################################

try:
    from id import MY_ID
except:
    MY_ID = 3

# Log settings
logging.basicConfig(format='%(asctime)s, %(levelname)s, %(message)s',datefmt='%H:%M:%S', level=logging.DEBUG)

# Start data thread
camera_thread = CameraUDP()
camera_thread.start()

# Configure the devices
ballsensor = BallSensor('in4')
base = DriveBase(left=('outC', Motor.POLARITY_INVERSED),
                 right=('outB', Motor.POLARITY_INVERSED),
                 wheel_diameter=4.3,
                 wheel_span=12,
                 counter_clockwise_is_positive=False) 
picker = Picker('outA')
battery = Battery()

# States
FLOCKING = 0  # For now, just behavior that makes robots avoid one another
SEEK_BALL = 1
PRE_STORE = 2
STORE = 3
TO_DEPOT = 4
PURGE = 5
LOW_VOLTAGE = 10
EXIT = 11

state = FLOCKING


#################################################################
###### At every time step, read camera data, process it,
###### and steer robot accordingly
#################################################################

while True:
    #################################################################
    ###### Receive data
    #################################################################
    logging.debug("Loop start")
    loopstart = time.time()
    try:
        # Get robot positions and settings from server
        data = camera_thread.get_data()

        # Get the data. Automatic exception if no data is available for MY_ID
        neighbor_info, robot_settings = data['neighbor_info'][MY_ID], data['robot_settings']
        # Unpack some useful data from the information we received
        neighbors = neighbor_info.keys()
        spring_between_robots = Spring(robot_settings['spring_between_robots'])
        my_gripper = robot_settings['p_bot_gripper']

    except:
        # Stop the loop if we're unable to get server data
        logging.warning("No data or the camera didn't see me. Waiting 1s")
        base.stop()
        time.sleep(1)
        continue
    logging.debug(str(time.time()-loopstart) + " Got data")

    if state == EXIT:
        camera_thread.stop()
        base.stop()
        break

    if state == FLOCKING:
        #################################################################
        ###### Process received data
        #################################################################

        total_force = np.array([0, 0])
        for neighbor in neighbors:
            neighbor_gripper = neighbor_info[neighbor]['center_location']
            spring_attachment = neighbor_gripper - np.array(robot_settings['p_bot_gripper'])
            total_force = total_force + spring_between_robots.get_force_vector(spring_attachment)

        #################################################################
        ###### Actuation based on processed data
        #################################################################

        # Decompose stretch into forward and sideways force
        sideways_force, forward_force = total_force

        logging.debug(str(time.time() - loopstart) + "Done spring calculations")

        # Obtain speed and turnrate
        speed = forward_force * robot_settings['speed_per_unit_force']
        turnrate = sideways_force * robot_settings['turnrate_per_unit_force']

    if state in (FLOCKING, SEEK_BALL):
        # Check for balls
        if ballsensor.ball_detected():
            logging.debug('ball detected')
            base.stop()
            picker.go_to_target(picker.STORE, blocking=True)
            picker.go_to_target(picker.OPEN, blocking=True)

    # Drive!
    base.drive_and_turn(speed, turnrate)

    # Debug print of where we're going        
    logging.debug('speed: ' + str(speed) + ' turnrate: ' + str(turnrate))

    #################################################################
    ###### Pause and repeat
    #################################################################
    time.sleep(0.1)
