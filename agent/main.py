#!/usr/bin/env python3

from camera_client import CameraUDP
from lightvectors.lightvectors import vector
import time
import logging
from hardware.motors import DriveBase, Picker
from hardware.sensors import BallSensor, Battery
from springs import Spring
from ball_sensor_reader import BallSensorReader

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
# camera_thread.start()

# Configure the devices
# ballsensor = BallSensor('in4')
ballsensor = BallSensorReader()
ballsensor.start()
base = DriveBase(left=('outC', DriveBase.POLARITY_INVERSED),
                 right=('outB', DriveBase.POLARITY_INVERSED),
                 wheel_diameter=4.3,
                 wheel_span=12,
                 counter_clockwise_is_positive=False) 
picker = Picker('outA')
battery = Battery()


# States
FLOCKING = 'flocking'  # For now, just behavior that makes robots avoid one another
SEEK_BALL = 'seek ball'
PRE_STORE = 'pre store'
STORE = 'store'
TO_DEPOT = 'to depot'
PURGE = 'purge'
LOW_VOLTAGE = 'low'
EXIT = 'exit'

state = FLOCKING

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
        data = camera_thread.read_from_socket()

        # Get the data. Automatic exception if no data is available for MY_ID
        neighbor_info, robot_settings = data['neighbors'], data['settings']
        wall_info, ball_info = data['walls'], data['balls']
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
        logging.warning("No data on my UDP port. Waiting 1s")
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
    force_to_top = spring_to_walls.get_force_vector(distance_to_top * world_y_in_my_frame)
    force_to_bottom = spring_to_walls.get_force_vector(-distance_to_bottom * world_y_in_my_frame)
    force_to_left = spring_to_walls.get_force_vector(-distance_to_left * world_x_in_my_frame)
    force_to_right = spring_to_walls.get_force_vector(distance_to_right * world_x_in_my_frame)

    # Make sum of total wall force
    nett_wall_force = force_to_top + force_to_bottom + force_to_left + force_to_right

    # 3. Nearest ball
    if number_of_balls > 0:
        nearest_ball = vector(ball_info[0])
        nett_ball_force = spring_to_balls.get_force_vector(nearest_ball)
    else:
        nett_ball_force = no_force

    logging.debug("Done spring calculations after {0}ms".format(int( (time.time()-loopstart)*1000 )))

    #################################################################
    ###### Strategy & state machine
    #################################################################

    # Do stuff with nett_ball_force, nett_neighbor_force and nett_wall_force, depending on where we want to go.
    total_force = no_force
    total_force = nett_neighbor_force
    #
    # if state == EXIT:
    #     camera_thread.stop()
    #     base.stop()
    #     break
    #
    # # Neighbor avoidance, but only in these states
    # if state in (FLOCKING, SEEK_BALL):
    #     total_force = total_force + nett_neighbor_force
    #
    # # Wall avoidance, but only in these states
    # if state in (FLOCKING, SEEK_BALL):
    #     total_force = total_force + nett_wall_force
    #
    # # Eat any ball we might accidentally see
    # if state in (FLOCKING, SEEK_BALL):
    #     if ballsensor.ball_detected() and not picker.is_running:
    #         picker.go_to_target(picker.STORE, blocking=False)
    #     logging.debug("Checked ball sensor after {0}ms".format(int((time.time() - loopstart) * 1000)))
    #
    # # Return picker to starting position after store, but only in these states
    # if state in (FLOCKING, SEEK_BALL):
    #     if picker.is_at_store:
    #         picker.go_to_target(picker.OPEN)
    #     logging.debug("Checked picker open after {0}ms".format(int((time.time() - loopstart) * 1000)))
    #
    # # Ball seeking regimen
    # if state == SEEK_BALL:
    #     # Check for balls
    #     total_force = total_force + nett_ball_force
    #     if nett_ball_force.norm < 5:  # TODO Make this a setting
    #         prestore_nett_ball_force = nett_ball_force
    #         prestore_start_time = time.time()
    #         state = PRE_STORE
    #
    # # When the ball is close, drive towards it blindly
    # # Until timeout or ball detection
    # if state == PRE_STORE:
    #     # Check for balls
    #     total_force = total_force + prestore_nett_ball_force
    #     if ballsensor.ball_detected() or time.time() > prestore_start_time + 1: # TODO also make this a setting
    #         picker.go_to_target(picker.STORE, blocking=False)
    #         # On to the next one
    #         state = SEEK_BALL
    #     logging.debug("Checked ball sensor after {0}ms".format(int((time.time() - loopstart) * 1000)))

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
    # time.sleep(0.1)
    logging.debug("Loop done. Speed:{0}, Turnrate:{1} ({3}), Looptime: {2}ms".format(speed,
                                                                                     turnrate,
                                                                                     int((time.time()-loopstart)*1000),
                                                                                     type(turnrate)
                                                                                     ))