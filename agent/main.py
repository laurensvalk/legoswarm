#!/usr/bin/env python3

from lightvectors.lightvectors import vector
import time
import logging
from hardware.motors import DriveBase, Picker
from hardware.simple_device import PowerSupply
from springs import Spring
from ball_sensor_reader import BallSensorReader
import socket, pickle, gzip, sys

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
TO_DEPOT = 'to depot'
PURGE = 'purge'
LOW_VOLTAGE = 'low'
EXIT = 'exit'

state = SEEK_BALL

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
        spring_between_robots = Spring(robot_settings['spring_between_robots'])
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

    nett_neighbor_force = no_force
    for neighbor in neighbors:
        neighbor_center = vector(neighbor_info[neighbor]['center_location'])
        spring_extension = neighbor_center - my_gripper
        nett_neighbor_force = nett_neighbor_force + spring_between_robots.get_force_vector(spring_extension)

    # 2. Walls

    # Unpack wall x and y directions, from my point of view
    world_x_in_my_frame, world_y_in_my_frame = vector(wall_info['world_x']), vector(wall_info['world_y'])

    # Unpack the distance to each wall, seen from my gripper
    (distance_to_top, distance_to_bottom, distance_to_left, distance_to_right) = wall_info['distances']

    # Make one spring to each wall
    # TODO check if springs are attached to my center or my gripper...
    force_to_top = spring_to_walls.get_force_vector(distance_to_top * world_y_in_my_frame)
    force_to_bottom = spring_to_walls.get_force_vector(-distance_to_bottom * world_y_in_my_frame)
    force_to_left = spring_to_walls.get_force_vector(-distance_to_left * world_x_in_my_frame)
    force_to_right = spring_to_walls.get_force_vector(distance_to_right * world_x_in_my_frame)

    # Make sum of total wall force
    nett_wall_force = force_to_top + force_to_bottom + force_to_left + force_to_right

    # 3. Nearest ball
    if number_of_balls > 0:
        ball_visible = True
        nearest_ball = vector(ball_info[0])
        nett_ball_force = spring_to_balls.get_force_vector(nearest_ball - my_gripper) #?
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
        total_force = total_force + nett_neighbor_force

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
            logging.debug("nearest ball at is {0}cm".format(nearest_ball.norm))
            if nearest_ball.norm < robot_settings['ball_close_enough']:
                state = STORE

    # When the ball is close, drive towards it blindly
    # Until timeout or ball detection
    if state == STORE:
        prestore_start_time = time.time()
        # First Point the robot straight towards the ball by zeroing the forward component
        if not (-2 < nearest_ball[0] < 2):
            total_force = [nett_ball_force[0]*2, 0]
        else:
            while not (time.time() > prestore_start_time + robot_settings['ball_grab_time'] or ballsensor.ball_detected()): #or ballsensor.ball_detected() ?
                base.drive_and_turn(4, 0)
            base.stop()
            picker.go_to_target(picker.STORE, blocking=True)
            picker.go_to_target(picker.OPEN, blocking=True)
            # On to the next one
            ball_count += 1
            if ball_count > 5:
                state = PURGE
            else:
                state = SEEK_BALL

    if state == PURGE:
        # Drive to a corner and purge
        corner_a_direction = vector(wall_info['corners'][0])
        total_force = spring_to_balls.get_force_vector(corner_a_direction) + nett_wall_force
        if corner_a_direction.norm < 20:
            base.stop()
            picker.go_to_target(picker.PURGE, blocking=True)
            picker.go_to_target(picker.OPEN, blocking=True)
            ball_count = 0
            time.sleep(1)
            state = SEEK_BALL

    logging.debug("State is "+str(state))
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
