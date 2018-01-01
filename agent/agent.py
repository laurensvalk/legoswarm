#!/usr/bin/env python3

from camera_client import CameraUDP
import numpy as np
import time
import logging
from hardware import DriveBase
from robot_frames import transform_to_world_from_camera, transform_to_world_from_bot

# My ID. Ultimately needs to come from elsewhere. E.g. Brick ID
MY_ID = 3

# Get default data in case no server is running
data = {'markers': {},
        'balls': [],
        'settings': {'sight_range': 300,
                     'dump_location': (20, 20),
                     'p_bot_midbase': (-4, -2),
                     'p_bot_gripper': (0, 5),
                     'field_height': 1080,
                     'field_width': 1920,
                     'cm_per_px': 0.1,
                     'speed_per_cm_spring_extension': 0.5 ,
                     'turnrate_per_cm_spring_extension': 5}
        }

# Log settings
logging.basicConfig(format='%(asctime)s, %(levelname)s, %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)

# Start data thread
camera_thread = CameraUDP()
camera_thread.start()

# Activate hardware if we're a robot
base = DriveBase(left='outB', right='outC', wheel_diameter=0.043, wheel_span=0.12)

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
        # Determine who's who
        agents = [i for i in markers]
        # List of everyone except me
        other_agents = [i for i in agents if i != MY_ID]

        # Empty dictionary of transformations from each robot to the world
        H_to_world_from_bot = {}

        # For every robot in the dataset, including me, determine position and orientation
        for i, [midbase_marker, apex_marker] in markers.items():
            # Get transformation matrix from pixels to world frame
            H_to_world_from_camera = transform_to_world_from_camera(settings)

            # Transform marker locations to world coordinates
            p_world_midbase_marker = H_to_world_from_camera*midbase_marker
            p_world_apex_marker = H_to_world_from_camera*apex_marker

            # Obtain transformation matrix between the robot and the world
            H_to_world_from_bot[i] = transform_to_world_from_bot(settings, p_world_midbase_marker, p_world_apex_marker)

        # Empty dictionary of neighboring gripper locations, in my frame of reference
        p_me_neighborgrippers = {}
        neighbors = []

        # Determine gripper location of everyone else in my reference frame
        for i in other_agents:
            # First, obtain the constant location a gripper in a robot frame
            p_bot_gripper = np.array(settings['p_bot_gripper'])

            # Transformation from another robot, to my reference frame
            H_to_me_from_otherbot = H_to_world_from_bot[MY_ID].inverse()@H_to_world_from_bot[i]

            # Gripper location of other robot, in my reference frame:
            p_me_othergripper = H_to_me_from_otherbot*p_bot_gripper

            # If that other gripper is in our field of view,
            # we consider it a neighbor and store the result
            if np.linalg.norm(p_me_othergripper) < settings['sight_range']:
                neighbors.append(i)
                p_me_neighborgrippers[i] = p_me_othergripper
                logging.debug("I see a neighbor gripper at: " + str(p_me_othergripper))

        # Now that we know the neighboring gripper positions, we can do something useful with them
        # For now, let us consider linear springs between all the grippers to achieve rendezvous.

        # Linear spring, sum of neighbor distances, otherwise 0
        sum_of_springs = np.array([0,0])
        for i in neighbors:
            sum_of_springs = sum_of_springs + p_me_neighborgrippers[i]

        # Decompose stretch into forward and sideways force
        forward_stretch, left_stretch = sum_of_springs[1], -sum_of_springs[0]

        # Obtain speed and turnrate
        speed =  forward_stretch * settings['turnrate_per_cm_spring_extension']
        turnrate = left_stretch * settings['speed_per_cm_spring_extension']

        # Drive!
        base.DriveAndTurn(speed,turnrate)

        # Debug print of where we're going        
        logging.debug('speed: ' + str(speed) + ' turnrate: ' + str(turnrate))

    # Pause after processing data
    time.sleep(1)            

# Closing down and cleaning up
camera_thread.stop()
