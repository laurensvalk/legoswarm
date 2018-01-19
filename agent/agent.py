#!/usr/bin/env python3

from camera_client import CameraUDP
import numpy as np
import time
import logging
from hardware import DriveBase
# from hardware_old import DriveBase
from springs import Spring

# My ID. Ultimately needs to come from elsewhere. E.g. Brick ID
try:
    from id import MY_ID
except:
    MY_ID = 3

# Log settings
logging.basicConfig(format='%(asctime)s, %(levelname)s, %(message)s',datefmt='%H:%M:%S', level=logging.WARNING)

# Start data thread
camera_thread = CameraUDP()
camera_thread.start()

# Activate hardware if we're a robot
base = DriveBase(left='outB', right='outC', wheel_diameter=0.043, wheel_span=0.12)


# Every time step, read camera data, process it, and steer robot accordingly
while True:
    t = time.time()
    logging.debug("Loop start")
    try:
        # Get robot positions and settings from server
        data = camera_thread.get_data()
        markers, balls, settings, localdata = data['markers'], data['balls'], data['settings'], data['localdata']
        spring_between_robots = Spring(settings['spring_between_robots'])
    except:
        # Stop the loop if we're unable to get server data
        logging.warning("No data. Waiting 1s")
        base.stop()
        time.sleep(1)
        continue

    logging.debug(str(time.time()-t) + " Got data")
    # Before doing anything, make sure the camera saw me. 
    # TODO: This currently relies on markers. Remove dependency; use only localdata
    if MY_ID not in markers:
        logging.warning("I am lost. Please send rescue.")
        base.stop()
    else:
        # Read out the neighbor gripper positions in my frame of reference
        p_me_neighborgrippers = localdata['neighborgrippers'][MY_ID]

        # Linear spring, sum of neighbor distances, otherwise 0
        total_force = np.array([0, 0])
        for (neighbor, gripper) in p_me_neighborgrippers.items():
            total_force = total_force + spring_between_robots.get_force_vector(gripper)    

        # Decompose stretch into forward and sideways force
        forward_force, sideways_force = total_force

        logging.debug(str(time.time() - t) + "Done spring calculations")

        # Obtain speed and turnrate
        # TODO: remove MINUS sign in turn rate below. Instead make CW/CCW a configurable option in drivebase
        speed = forward_force * settings['speed_per_unit_force']
        turnrate = -sideways_force * settings['turnrate_per_unit_force']
        logging.debug("Speed: {0} = {1} stretch x {2} rate".format(speed, forward_force,
                                                                   settings['speed_per_unit_force']))
        logging.debug("Turnrate: {0} = {1} stretch x {2} rate".format(turnrate, sideways_force,
                                                                   settings['turnrate_per_unit_force']))

        # Drive!
        base.drive_and_turn(speed, turnrate)

        # Debug print of where we're going        
        logging.debug('speed: ' + str(speed) + ' turnrate: ' + str(turnrate))

    # Pause after processing data
    # time.sleep(1)

# Closing down and cleaning up
camera_thread.stop()
