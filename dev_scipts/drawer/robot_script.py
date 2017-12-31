#!/usr/bin/env python3

import socket, sys
from threading import Thread
import logging
from math import sin, cos
import numpy as np
import ev3dev.auto as ev3

try:
    import cPickle as pickle
except:
    import pickle

### Initialize ###
# Constants
PORT = 50008
THIS_ROBOT = 2  # Our own ID
PI = 3.1415
TWO_PI = 2 * PI
CIRCLE = 1
GO_TO_CENTER = 0
CENTER = np.array([1920 / 2, 1080 / 2])
MODE = CIRCLE     # GO_TO_CENTER or CIRCLE

# Server communication
robot_broadcast_data = {'states': {}, 'balls': {}, 'settings': {}}
running = True
logging.basicConfig(  # filename='position_server.log',     # To a file. Or not.
    filemode='w',  # Start each run with a fresh log
    format='%(asctime)s, %(levelname)s, %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO, )  # Log info, and warning

# Robot setup
left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
right_motor = ev3.LargeMotor(ev3.OUTPUT_C)


### Helpers ###
def vec2d_length(vector):
    """
    Calculates the length of a 2D vector

    :param vector: 1 x 2 numpy array
    :return: length (float)
    """
    return np.dot(vector, vector) ** 0.5


def vec2d_angle(vector):
    """
    Calculates the angle of a vector with the horizontal axis (X),
    in right handed Cartesian space. (A turn to the left is positive, Y axis pointing up).

    :param vector: 1 x 2 numpy array
    :return: angle in radians (float)
    """
    return np.arctan2(vector[1], vector[0])


def clamp(n, range):
    """
    Given a number and a range, return the number, or the extreme it is closest to.

    :param n: number
    :param range: tuple with min and max
    :return: number
    """
    minn, maxn = range
    return max(min(maxn, n), minn)


def circle_steps(origin, radius, step_px):

    circumference = radius * 2 * PI
    numsteps = int(circumference / step_px) + 1
    i = 0
    while True:
        angle = 2 * PI / numsteps * i
        coord = np.array([cos(angle) * radius + origin[0], sin(angle) * radius + origin[1]])
        yield coord
        i += 1
        if i > numsteps: i = 0


### Get robot positions from server ###
def get_camera_data():
    global robot_broadcast_data, running
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', PORT))

    while running:
        try:
            data, server = s.recvfrom(2048)
            robot_broadcast_data = pickle.loads(data)
        except:
            e = sys.exc_info()[0]
            logging.warning(e)
            raise

    s.close()


### Run the main loop ###
if __name__ == '__main__':
    get_positions_thread = Thread(target=get_camera_data)
    get_positions_thread.start()
    circle_step = circle_steps((1920//2, 1080//2), 400, 50)

    if MODE == CIRCLE:
        target = next(circle_step)
    else:
        target = CENTER
    logging.info("Current target:" + str(target))

    logging.info("Running")
    while 1:
        try:
            # We put this in a try statement because we need to clean up after ctrl-c

            if THIS_ROBOT in robot_broadcast_data['states']:
                # Copy global variable to local ones, otherwise Bad Shit Happens in MultiThreading
                center, nose = np.array(robot_broadcast_data['states'][THIS_ROBOT])
                heading = vec2d_angle(nose - center)

                # Calculate vector from nose to target
                path = target - nose

                if MODE == CIRCLE:
                    if vec2d_length(path) <= 6:
                        try:
                            target = next(circle_step)
                            logging.info("Current target:" + str(target))
                        except:
                            break  # no more points to be had
                        path = target - nose

                target_direction = vec2d_angle(path) - heading
                turnrate = clamp(vec2d_length(path) * sin(target_direction) * -1, (-500, 500))
                speed = clamp(vec2d_length(path) * cos(target_direction) * -2, (-500, 500))

                left_motor.run_forever(speed_sp=(speed + turnrate))
                right_motor.run_forever(speed_sp=(speed - turnrate))
            else:
                # No data about our robot's position. Stop the engines!
                left_motor.stop()
                right_motor.stop()

        except:
            # User pressed ctrl-c or something bad happened
            e = sys.exc_info()[0]
            logging.warning(e)
            running = False
            left_motor.stop()
            right_motor.stop()
            logging.info("Cleaned up")
            raise

    # Clean up
    running = False
    left_motor.stop()
    right_motor.stop()
    logging.info("Cleaned up")
