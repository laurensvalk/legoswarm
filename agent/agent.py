#!/usr/bin/env python3

from camera_client import CameraUDP
from numpy import array
import time
import logging

from robot_frames import transform_to_world_from_camera, transform_to_world_from_bot

# My ID. Ultimately needs to come from elsewhere. E.g. Brick ID
MY_ID = 1

# Log settings
logging.basicConfig(format='%(asctime)s, %(levelname)s, %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)

# Start data thread
camera_thread = CameraUDP()
camera_thread.start()

# Every time step, read camera data, process it, and steer robot accordingly
for timeindex in range(0,50):
    try:
        # Get robot positions and settings from server
        data = camera_thread.get_data()
        markers, balls, settings = data['markers'], data['balls'], data['settings']        
    except:
        # Stop the loop if we're unable to get server data
        break
    
    # Before doing anything, make sure the camera saw me. 
    if MY_ID not in markers:
        logging.warning("I am lost. Please send rescue.")
    else:
        # For every robot in the dataset, including me, determine position and orientation
        for i, [midbase_marker, apex_marker] in markers.items():
            # Get transformation matrix from pixels to world frame
            H_to_world_from_camera = transform_to_world_from_camera(settings)

            # Transform marker locations to world coordinates
            p_world_midbase_marker = H_to_world_from_camera*midbase_marker
            p_world_apex_marker = H_to_world_from_camera*apex_marker

            # Obtain transformation matrix between the robot and the world
            H_to_world_from_bot = transform_to_world_from_bot(settings, p_world_midbase_marker, p_world_apex_marker)

            # Obtain gripper position in the world frame by transforming the location the bot frame:
            p_bot_gripper = array(settings['p_bot_gripper'])
            p_world_gripper = H_to_world_from_bot*p_bot_gripper

            if i == MY_ID:
                logging.debug("My gripper should be here")
                logging.debug(p_world_gripper)

    # Pause after processing data
    time.sleep(1)            

# Closing down and cleaning up
camera_thread.stop()        