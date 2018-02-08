#!/usr/bin/env python3

from lightvectors.lightvectors import vector
import time
import logging
from hardware.motors import DriveBase, Picker
from hardware.simple_device import PowerSupply, Buttons
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
logging.basicConfig(format='%(asctime)s, %(levelname)s, %(message)s',datefmt='%H:%M:%S', level=logging.INFO)

# Start data thread
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 50000+MY_ID
s.bind(('', 50000+MY_ID))
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1500)
s.settimeout(0.1)
logging.debug("Listening on port {0}".format(port))

# Configure the devices
ballsensor = BallSensorReader()
ballsensor.start()
base = DriveBase(left=('outC', DriveBase.POLARITY_INVERSED),
                 right=('outB', DriveBase.POLARITY_INVERSED),
                 wheel_diameter=4.3,
                 wheel_span=12,
                 counter_clockwise_is_positive=False) 
picker = Picker('outA')
battery = PowerSupply()
buttons = Buttons()

# States
FLOCKING = 'flocking'  # For now, just behavior that makes robots avoid one another
SEEK_BALL = 'seek ball'
STORE = 'store'
TO_DEPOT = 'to depot'
PURGE = 'purge'
LOW_VOLTAGE = 'low'
EXIT = 'exit'
BOUNCE = 'bounce'
DRIVE = 'drive'
PAUSE = 'pause'
TO_CENTER = 'to_center'

pause_end_time = time.time()
pause_next_state = SEEK_BALL

state = DRIVE
CHECK_VOLT_AFTER_LOOPS = 500
MAX_FAILS_BEFORE_WAIT = 8
loopcount = 0
failcount = 0
last_volt_check = time.time()
no_force = vector([0, 0])

#################################################################
###### At every time step, read camera data, process it,
###### and steer robot accordingly
#################################################################

while True:

    logging.debug("Loop start")
    loopstart = time.time()
    loopcount += 1

    #################################################################
    ###### Receive data
    #################################################################

    try:
        # Get robot positions and settings from server
        compressed_data, server = s.recvfrom(1500)
        data = pickle.loads(gzip.decompress(compressed_data))

        # Get the data. Automatic exception if no data is available for MY_ID
        neighbor_info = data['neighbors']
        robot_settings = data['robot_settings']
        wall_info = data['walls']
        ball_info = data['balls']

        # Unpack some useful data from the information we received
        neighbors = neighbor_info.keys()
        my_gripper = vector(robot_settings['p_bot_gripper'])
        my_center = vector([0,0])
        my_tail = -my_gripper

        # Check how many balls are near me
        number_of_balls = len(ball_info)
        
        # Unpack spring characteristics
        robot_avoidance_spring = Spring(robot_settings['robot_avoidance_spring'])
        robot_attraction_spring = Spring(robot_settings['robot_attraction_spring'])
        spring_to_walls = Spring(robot_settings['spring_to_walls'])
        spring_to_balls = Spring(robot_settings['spring_to_balls'])
        spring_to_position = Spring(robot_settings['spring_to_position'])
        if 'state' in robot_settings:
            if robot_settings['state']:
                state = robot_settings['state']

    except Exception as e:
        # Stop the loop if we're unable to get server data
        logging.warning("{0}: Reading data from port {1} failed. Waiting...".format(repr(e), port))

        if 'backspace' in buttons.buttons_pressed:
            ballsensor.stop()
            break
        if failcount > MAX_FAILS_BEFORE_WAIT:
            base.stop()
            time.sleep(0.5)
        failcount += 1
        continue

    failcount = 0
    logging.debug("Got data after {0}ms".format(int( (time.time()-loopstart)*1000 )))

    #################################################################
    ###### All calculations
    #################################################################

    # 1. Neighbors

    nett_neighbor_avoidance = no_force
    nett_neighbor_attraction = no_force
    for neighbor in neighbors:
        neighbor_center = vector(neighbor_info[neighbor]['center_location'])
        neighbor_gripper = vector(neighbor_info[neighbor]['gripper_location'])
        neighbor_tail = neighbor_center + (neighbor_center - neighbor_gripper)

        # Now we compare my gripper, center, and tail to every point on the neighbor
        # The closest one will pose the most immediate threat for collision.

        # Initialize closest point at infinity
        shortest_spring_length = 100000

        # Loop over all 9 point combinations to find the most threatening one
        for neighbor_point in (neighbor_gripper, neighbor_center, neighbor_tail):
            for my_point in (my_gripper, my_center, my_tail):
                difference = (neighbor_point-my_point).norm
                if difference < shortest_spring_length:
                    shortest_spring_length = difference

        # As the spring direction, we always take the spring to the neighbor center, but use the length from above
        avoidance_direction = (neighbor_center-my_gripper).unit
        nett_neighbor_avoidance += robot_avoidance_spring.get_force_vector(avoidance_direction*shortest_spring_length)

        # ... and add attraction springs only to their centers
        if neighbor == 1:
            nett_neighbor_attraction = nett_neighbor_attraction + robot_attraction_spring.get_force_vector(neighbor_center - my_gripper)

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

    # 4. Start with a zero total force for processing all state behaviour
    total_force = no_force

    logging.debug("Done spring calculations after {0}ms".format(int( (time.time()-loopstart)*1000 )))

    #################################################################
    ###### Strategy & state machine
    #################################################################

    # Do stuff with nett_ball_force, nett_neighbor_force and nett_wall_force, depending on where we want to go.

    if loopcount > CHECK_VOLT_AFTER_LOOPS:
        loopcount = 0
        voltage = battery.voltage
        logging.info("Battery is at {0}V".format(voltage))
        if voltage < 7.2:
            state = LOW_VOLTAGE

    # Go to depot if our belly is full.
    if state in (SEEK_BALL, DRIVE, BOUNCE, ):
        if picker.store_count > robot_settings['max_balls_in_store']:
            state = PURGE
            logging.info("Changing to {0} state".format(state))

    # Drive to field corner c when voltage is low.
    if state == LOW_VOLTAGE:
        corner_c_direction = vector(wall_info['corners'][2])
        total_force = spring_to_position.get_force_vector(corner_c_direction) + nett_wall_force

    # Stop this program. Not used, so far.
    if state == EXIT:
        base.stop()
        ballsensor.stop()
        break

    # Eat any ball we might see
    if state in (BOUNCE, DRIVE,):
        detected = ballsensor.ball_detected()
        logging.debug("Checked ball sensor after {0}ms. Distance: {1}".format(int((time.time() - loopstart) * 1000),
                                                                              ballsensor.distance))
        if detected and not picker.is_running:
            base.stop()
            picker.store()
            time.sleep(0.5)
            picker.open(blocking=True)
            time.sleep(0.5)
            # Clear the buffer so we have up-to-date data at the next loop
            try:
                compressed_data, server = s.recvfrom(1500)
            except:
                pass

    # Flocking regimen
    if state == FLOCKING:
        total_force = nett_wall_force + nett_neighbor_avoidance + nett_neighbor_attraction

    # Ball seeking regimen
    if state == SEEK_BALL:
        total_force = nett_neighbor_avoidance + nett_wall_force + nett_ball_force
        if ball_visible:
            logging.debug("nearest ball at is {0}cm, {1}".format(nearest_ball_to_my_gripper.norm, nearest_ball_to_my_gripper))
            if nearest_ball_to_my_gripper.norm < robot_settings['ball_close_enough']:
                total_force = no_force
                state = STORE
                logging.info("Changing to {0} state".format(state))

    # When the ball is close, drive towards it blindly
    if state == STORE:
        vector_to_ball = nearest_ball_to_my_gripper + my_gripper
        angle_to_ball = vector_to_ball.angle_with_y_axis * 180/3.1415
        distance_to_ball = vector_to_ball.norm - my_gripper.norm
        logging.debug(
            "Storing with turn: {0}, distance: {1}, stored:{2}".format(angle_to_ball,
                                                                       robot_settings['ball_close_enough'],
                                                                       picker.store_count))
        time.sleep(2)

        # Drive to the ball's last position
        base.turn_degrees(angle_to_ball)
        base.drive_cm(distance_to_ball)

        # The ball should be right in the gripper now.
        picker.store()
        picker.open()

        # Clear the buffer so we have up-to-date data at the next loop
        try:
            compressed_data, server = s.recvfrom(1500)
        except:
            pass

        # Next state
        state = PAUSE
        logging.info("Changing to {0} state".format(state))
        pause_end_time = time.time() + 2
        pause_next_state = SEEK_BALL

    if state == PAUSE:
        total_force = no_force
        if time.time() > pause_end_time:
            state = pause_next_state
            logging.info("Changing to {0} state".format(state))

    if state == PURGE:
        # Drive to a corner and purge
        mid_of_a_d = vector((vector(wall_info['corners'][0]) + vector(wall_info['corners'][3])) / 2)
        total_force = spring_to_position.get_force_vector(mid_of_a_d) + nett_neighbor_avoidance
        picker.store()
        if mid_of_a_d.norm < robot_settings['distance_to_purge_location']:
            base.stop()
            picker.purge()
            time.sleep(1)
            picker.store()

            # Clear the buffer so we have up-to-date data at the next loop
            try:
                compressed_data, server = s.recvfrom(1500)
            except:
                pass

            state = TO_CENTER
            logging.info("Changing to {0} state".format(state))

    if state == TO_CENTER:
        center_direction = vector((vector(wall_info['corners'][0]) + vector(wall_info['corners'][2])) / 2)
        total_force = spring_to_position.get_force_vector(center_direction) + nett_neighbor_avoidance
        if center_direction.norm < 40:
            picker.open()
            state = DRIVE
            logging.info("Changing to {0} state".format(state))

    if state == DRIVE:
        total_force = nett_neighbor_avoidance + vector([0, robot_settings['bounce_drive_speed']])
        if min(wall_info['distances']) < robot_settings['min_wall_distance']:
            # random_factor = 1+(random.random()/10)
            state = BOUNCE
            logging.info("Changing to {0} state".format(state))

    if state == BOUNCE:
        # total_force = nett_neighbor_avoidance + vector([nett_wall_force[0]*random_factor, nett_wall_force[1]])  # yuck
        total_force = nett_neighbor_avoidance + nett_wall_force
        if min(wall_info['distances']) > 20:
            state = DRIVE
            logging.info("Changing to {0} state".format(state))

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
