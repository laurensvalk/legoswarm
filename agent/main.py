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
camera_thread = CameraUDP(port=50000+MY_ID)
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

no_force = np.array([0, 0])

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
        neighbor_info = data['neighbors']
        robot_settings = data['settings']
        wall_info = data['walls'] 
        ball_info = data['balls']
        # Unpack some useful data from the information we received
        neighbors = neighbor_info.keys()
        my_gripper = np.array(robot_settings['p_bot_gripper'])

        # Check how many balls are near me
        number_of_balls = len(ball_info)
        
        # Unpack spring characteristics
        spring_between_robots = Spring(robot_settings['spring_between_robots'])
        spring_to_walls = Spring(robot_settings['spring_to_walls'])   
        spring_to_balls = Spring(robot_settings['spring_to_balls'])   

    except:
        # Stop the loop if we're unable to get server data
        logging.warning("No data on my UDP port. Waiting 1s")
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
        ###### Process Neighbor info
        #################################################################

        total_force = no_force
        for neighbor in neighbors:
            neighbor_center = np.array(neighbor_info[neighbor]['center_location'])
            spring_extension = neighbor_center - my_gripper
            total_force = total_force + spring_between_robots.get_force_vector(spring_extension)

        #################################################################
        ###### Process Wall info
        #################################################################            
        
        # Unpack wall x and y directions, from my point of view
        world_x_in_my_frame, world_y_in_my_frame = np.array(wall_info['world_x']), np.array(wall_info['world_y'])

        # Unpack the distance to each wall, seen from my gripper
        (distance_to_top, distance_to_bottom, distance_to_left, distance_to_right) = wall_info['distances']
        
        # Make one spring to each wall
        force_to_top = spring_to_walls.get_force_vector(distance_to_top*world_y_in_my_frame)
        force_to_bottom = spring_to_walls.get_force_vector(-distance_to_bottom*world_y_in_my_frame)
        force_to_left = spring_to_walls.get_force_vector(-distance_to_left*world_x_in_my_frame)
        force_to_right = spring_to_walls.get_force_vector(distance_to_right*world_x_in_my_frame)

        # Make sum of total wall force
        nett_wall_force = force_to_top + force_to_bottom + force_to_left + force_to_right

        # TODO Decide what to do with it later, given the state machine. For now just add it.
        total_force = total_force + nett_wall_force

        #################################################################
        ###### Process Ball info
        #################################################################            
                
        if number_of_balls > 0:
            nearest_ball = np.array(ball_info[0])
            ball_force = spring_to_balls.get_force_vector(nearest_ball)
        else:
            ball_force = no_force

        #################################################################
        ###### Actuation based on processed data
        #################################################################

        # Decompose stretch into forward and sideways force
        sideways_force, forward_force = total_force

        logging.debug(str(time.time() - loopstart) + "Done spring calculations")

        # Obtain speed and turnrate
        speed = forward_force * robot_settings['speed_per_unit_force']
        turnrate = sideways_force * robot_settings['turnrate_per_unit_force']

    if state in (SEEK_BALL,):
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
