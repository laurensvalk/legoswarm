#!/usr/bin/env python3

from camera_client import CameraUDP
import numpy as np
import time
import logging
from hardware import DriveBase

# My ID. Ultimately needs to come from elsewhere. E.g. Brick ID
MY_ID = 3

# Log settings
logging.basicConfig(format='%(asctime)s, %(levelname)s, %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)

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
        markers, localdata, balls, settings = data['markers'], data['balls'], data['settings'], data['localdata']
    except:
        # Stop the loop if we're unable to get server data
        logging.warning("No data. Waiting 1s")
        base.Stop()
        time.sleep(1)
        break

    logging.debug(str(time.time()-t) + "Got data")
    # Before doing anything, make sure the camera saw me. 
    # TODO: This currently relies on markers. Remove dependency; use only localdata
    if MY_ID not in markers:
        logging.warning("I am lost. Please send rescue.")
        base.Stop()
    else:
        # Read out the neighbor gripper positions in my frame of reference
        p_me_neighborgrippers = localdata['neighborgrippers'][MY_ID]

        # Linear spring, sum of neighbor distances, otherwise 0
        sum_of_springs = np.array([0,0])
        for i in p_me_neighborgrippers:
            sum_of_springs = sum_of_springs + p_me_neighborgrippers[i]

        # Decompose stretch into forward and sideways force
        forward_stretch, left_stretch = sum_of_springs[1], -sum_of_springs[0]

        logging.debug(str(time.time() - t) + "Done spring calculations")

        # Obtain speed and turnrate
        speed =  forward_stretch * settings['speed_per_cm_spring_extension']
        turnrate = left_stretch * settings['turnrate_per_cm_spring_extension']
        logging.debug("Speed: {0} = {1} stretch x {2} rate".format(speed, forward_stretch,
                                                                   settings['speed_per_cm_spring_extension']))
        logging.debug("Turnrate: {0} = {1} stretch x {2} rate".format(speed, left_stretch,
                                                                   settings['turnrate_per_cm_spring_extension']))

        # Drive!
        base.DriveAndTurn(speed,turnrate)

        # Debug print of where we're going        
        logging.debug('speed: ' + str(speed) + ' turnrate: ' + str(turnrate))

    # Pause after processing data
    # time.sleep(1)

# Closing down and cleaning up
camera_thread.stop()
