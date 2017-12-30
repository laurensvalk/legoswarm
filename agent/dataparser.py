#!/usr/bin/env python3

from numpy import array, append
from numpy.linalg import norm
from frames import Transformation, COL, ROW
import time
from camera_client import CameraUDP
from dummydata import read_broadcast     

# My ID. Ultimately needs to come from elsewhere. E.g. Brick ID
me = 1

def spring(spring_extension_b):
    """Convert spring elongation in body frame to forward speed and rotation rate"""

    forward_extension = spring_extension_b[0]
    medial_extension = spring_extension_b[1]

    # For now just linear relations. Eventually different pattern: repelling nearby, attractive afar, attenuating at infinity etc
    speed = forward_extension*1000
    turn_rate = medial_extension*1000

    return speed, turn_rate


camera_thread = CameraUDP()
camera_thread.start()

# Eventually we're going to do things in a loop. Now just a couple times for debugging
for timeindex in range(0,100):

    time.sleep(0.5)
    try:
        # Get robot positions from server
        data = camera_thread.get_data()
    except:
        # Time to panic, log some errors and kill others threads.
        camera_thread.stop()
        raise

    # Read the data
    markers, balls, settings = data['markers'], data['balls'], data['settings']

    # Before doing anything, make sure the camera saw me. 
    if me not in markers:
        pass
    else:

        ##
        ## Constant transformation from pixels to meters
        ##
        H_w_p = Transformation(array([[settings['eps'], 0],[0, -settings['eps']]]),
                                    array([-settings['eps']*settings['dpx']/2,settings['eps']*settings['dpy']/2]))


        ##
        ## For each robot, figure out the gripper positions in world coordinates
        ##

        # Constants for every robot
        p_midbase_robot = array(settings['p_b_midbase'])
        p_b_gripper = array(settings['p_b_gripper'])
        H_b_l = Transformation(array([[1,0],[0,1]]), p_midbase_robot)

        # Empty list of gripper locations
        p_w_gripper = {}

        for i, [p_p_midbasei, p_p_apexi] in markers.items():
            # Transform pixels to world frame
            p_w_midbasei = H_w_p*p_p_midbasei
            p_w_apexi = H_w_p*p_p_apexi

            # Transformation between label and world
            x_hat_w_li = (p_w_apexi-p_w_midbasei).reshape((2,1))/norm(p_w_apexi-p_w_midbasei)
            y_hat_w_li = array([[0, -1], [1, 0]])@x_hat_w_li
            H_w_li = Transformation(append(x_hat_w_li, y_hat_w_li, axis=COL), p_w_midbasei)    

            # Transformation between i'th robot and world
            H_bi_w = H_b_l@H_w_li.inverse()

            # Gripper of robot i in world coordinates
            p_w_gripper[i] = H_bi_w.inverse()*p_b_gripper

            # Also store some of the results for myself
            if i == me:
                H_b_w = H_bi_w

        ##
        ## For each robot that isn't me, determine if it is close enough to be considered my neighbor
        ##    
        
        # First, find the other grippers in my frame of reference
        other_agents = [i for i in markers if not i == me]
        p_b_gripperi = {i: H_b_w*p_w_gripper[i] for i in other_agents}

        # See which of these is nearby, and is therefore a neighbor
        neighbors = [i for i in other_agents if norm(p_b_gripperi[i]-p_b_gripper) < settings['sight_range']]

        # Print neighbor locations for debugging
        for i in neighbors:
            print(p_b_gripperi[i])

        ##
        ## Above, I processed camera data to get information that I would normally observe using sensors
        ## Below, I'll continue with my actual behavior based on this "observed data"
        ##

        ##
        ## Driving towards center of field while avoiding other robots:
        ##   
        
        # springs = {i: p_b_gripperi[i]-p_b_gripper for i in neighbors}                  



        # nett_force = {i: p_b_gripperi[i]-p_b_gripper for i in neighbors}

        # for i in neighbors:
        #     print(springs[i])

camera_thread.stop()        