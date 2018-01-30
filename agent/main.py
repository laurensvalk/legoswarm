#!/usr/bin/env python3

from lightvectors.lightvectors import vector
import time
import logging
from hardware.motors import DriveBase, Picker
from hardware.simple_device import PowerSupply
from springs import Spring
from ball_sensor_reader import BallSensorReader
import socket, pickle, gzip, sys
import random

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
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 50000+MY_ID
s.bind(('', 50000+MY_ID))
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1500)
s.settimeout(0.2)
logging.debug("Listening on port {0}".format(port))

# Configure the devices
# ballsensor = BallSensor('in4')
ballsensor = BallSensorReader()
ballsensor.start()
ball_count = 0

base = DriveBase(left=('outC', DriveBase.POLARITY_INVERSED),
                 right=('outB', DriveBase.POLARITY_INVERSED),
                 wheel_diameter=4.3,
                 wheel_span=12,
                 counter_clockwise_is_positive=False) 
picker = Picker('outA')
battery = PowerSupply()


# States
FLOCKING = 'flocking'  # For now, just behavior that makes robots avoid one another
SEEK_BALL = 'seek ball'
STORE = 'store'
PAUSE = 'pause'
pause_end_time = time.time()
TO_DEPOT = 'to depot'
PURGE = 'purge'
LOW_VOLTAGE = 'low'
EXIT = 'exit'
BOUNCE = 'bounce'
bounce_direction = random.random()*2*3.1415

state = BOUNCE

no_force = vector([0, 0])

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
        compressed_data, server = s.recvfrom(1500)
        data = pickle.loads(gzip.decompress(compressed_data))

        # Get the data. Automatic exception if no data is available for MY_ID
        neighbor_info = data['neighbors']
        robot_settings = data['settings']
        wall_info = data['walls']
        ball_info = data['balls']

        # Unpack some useful data from the information we received
        neighbors = neighbor_info.keys()
        my_gripper = vector(robot_settings['p_bot_gripper'])

        # Check how many balls are near me
        number_of_balls = len(ball_info)
        
        # Unpack spring characteristics
        push_spring_between_robots = Spring(robot_settings['push_spring_between_robots'])
        pull_spring_between_robots = Spring(robot_settings['pull_spring_between_robots'])
        spring_to_walls = Spring(robot_settings['spring_to_walls'])
        spring_to_balls = Spring(robot_settings['spring_to_balls'])   

    except:
        # Stop the loop if we're unable to get server data
        e = sys.exc_info()[0]
        logging.warning("{0}: Reading data from port {1} failed. Waiting 1s".format(e, port))
        base.stop()
        time.sleep(1)
        continue
    logging.debug("Got data after {0}ms".format(int( (time.time()-loopstart)*1000 )))

    #################################################################
    ###### All calculations
    #################################################################

    # 1. Neighbors

    nett_neighbor_avoidance = no_force
    nett_neighbor_attraction = no_force
    for neighbor in neighbors:
        neighbor_center = vector(neighbor_info[neighbor]['center_location'])
        spring_extension = neighbor_center - my_gripper
        nett_neighbor_avoidance = nett_neighbor_avoidance + pull_spring_between_robots.get_force_vector(spring_extension)
        nett_neighbor_attraction = nett_neighbor_attraction + push_spring_between_robots.get_force_vector(spring_extension)



    # 2. Walls

    # Unpack wall x and y directions, from my point of view
    world_x_from_my_gripper = vector(wall_info['world_x'])
    world_y_from_my_gripper = vector(wall_info['world_y'])

    # Unpack the distance to each wall, seen from my gripper
    (distance_to_top, distance_to_bottom, distance_to_left, distance_to_right) = wall_info['distances']

    # Make one spring to each wall
    force_to_top = spring_to_walls.get_force_vector(distance_to_top * world_y_from_my_gripper)
    force_to_bottom = spring_to_walls.get_force_vector(-distance_to_bottom * world_y_from_my_gripper)
    force_to_left = spring_to_walls.get_force_vector(-distance_to_left * world_x_from_my_gripper)
    force_to_right = spring_to_walls.get_force_vector(distance_to_right * world_x_from_my_gripper)

    # Make sum of total wall force
    nett_wall_force = force_to_top + force_to_bottom + force_to_left + force_to_right

    # 3. Nearest ball
    if number_of_balls > 0:
        ball_visible = True
        nearest_ball_to_my_gripper = vector(ball_info[0])
        nett_ball_force = spring_to_balls.get_force_vector(nearest_ball_to_my_gripper)
    else:
        ball_visible = False
        nett_ball_force = no_force

    logging.debug("Done spring calculations after {0}ms".format(int( (time.time()-loopstart)*1000 )))

    #################################################################
    ###### Strategy & state machine
    #################################################################

    # Do stuff with nett_ball_force, nett_neighbor_force and nett_wall_force, depending on where we want to go.
    total_force = no_force
    # total_force = nett_neighbor_force

    if state == EXIT:
        base.stop()
        break

    # Neighbor avoidance, but only in these states
    if state in (FLOCKING, SEEK_BALL,):
        total_force = total_force + nett_neighbor_avoidance

    # Neighbor attraction, but only in these states
    if state in (FLOCKING,):
        total_force = total_force + nett_neighbor_attraction

    # Wall avoidance, but only in these states
    if state in (FLOCKING, SEEK_BALL,):
        total_force = total_force + nett_wall_force

    # Eat any ball we might accidentally see
    if state in ('',):
        if ballsensor.ball_detected() and not picker.is_running:
            picker.go_to_target(picker.STORE, blocking=False)
        logging.debug("Checked ball sensor after {0}ms".format(int((time.time() - loopstart) * 1000)))

    # Return picker to starting position after store, but only in these states
    if state in (FLOCKING, SEEK_BALL):
        if picker.is_at_store:
            picker.go_to_target(picker.OPEN)
        logging.debug("Checked picker open after {0}ms".format(int((time.time() - loopstart) * 1000)))

    # Ball seeking regimen
    if state == SEEK_BALL:
        # Check for balls
        total_force = total_force + nett_ball_force
        if ball_visible:
            logging.debug("nearest ball at is {0}cm, {1}".format(nearest_ball_to_my_gripper.norm, nearest_ball_to_my_gripper))
            if nearest_ball_to_my_gripper.norm < robot_settings['ball_close_enough']:
                total_force = no_force
                state = STORE

    # When the ball is close, drive towards it blindly
    if state == STORE:
        vector_to_ball = nearest_ball_to_my_gripper + my_gripper
        angle_to_ball = vector_to_ball.angle_with_y_axis * 180/3.1415
        distance_to_ball = vector_to_ball.norm - my_gripper.norm
        base.turn_degrees(angle_to_ball)
        base.drive_cm(distance_to_ball)
        logging.debug(
            "Storing turn: {0}, distance: {1}".format(angle_to_ball, robot_settings['ball_close_enough']))

        # The ball should be right in the gripper now.
        # picker.go_to_target(picker.STORE)

        # Clear the buffer so we have up-to-date data at the next loop
        compressed_data, server = s.recvfrom(1500)

        # Next state
        state = PAUSE
        pause_end_time = time.time() + 5

    if state == PAUSE:
        if time.time() > pause_end_time:
            state = SEEK_BALL

    if state == PURGE:
        # Drive to a corner and purge
        corner_a_direction = vector(wall_info['corners'][0])
        total_force = spring_to_balls.get_force_vector(corner_a_direction) + nett_wall_force
        if corner_a_direction.norm < 20:
            base.stop()
            picker.go_to_target(picker.PURGE, blocking=True)
            picker.go_to_target(picker.OPEN, blocking=False)
            ball_count = 0
            time.sleep(1)
            state = SEEK_BALL

    if state == BOUNCE:
        base.drive_and_turn(2,0)
        if distance_to_top < 15 or \
            distance_to_bottom < 15 or \
            distance_to_left < 15 or \
            distance_to_right < 15:
                turn_left = random.randrange(1)
                if turn_left:
                    base.turn_degrees(random.randrange(90, 180))
                else:
                    base.turn_degrees(random.randrange(-90, -180))

    logging.debug("State strategy processed for state {0} after {1}ms".format(state,
                                                                              int((time.time()-loopstart)*1000)))

    #################################################################
    ###### Actuation based on processed data, state & strategy
    #################################################################

    # Decompose total force into forward and sideways force
    sideways_force, forward_force = total_force
    speed = forward_force * robot_settings['speed_per_unit_force']
    turnrate = sideways_force * robot_settings['turnrate_per_unit_force']
    base.drive_and_turn(speed, turnrate)
    # Time for pause is here
    # time.sleep(1)
    logging.debug("Loop done. Speed:{0:.2f}, Turnrate:{1:.2f}, Looptime: {2}ms".format(speed,
                                                                                     turnrate,
                                                                                     int((time.time()-loopstart)*1000),
                                                                                     total_force
                                                                                     ))
