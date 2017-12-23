#!/usr/bin/env python3

# Reads webcam data and outputs found triangle markers as an UDP broadcast
# Use q to stop this script, not ctrl-c!

import cv2
import numpy as np
import time
from threading import Thread
import socket
import logging
try:
    import cPickle as pickle
except:
    import pickle


### Settings ###
THRESH = 100        # Threshold for b/w version of camera image
SERVER_ADDR = ("255.255.255.255", 50008)



### Initialize ###
# Camera
cv2.namedWindow("cam", cv2.WINDOW_OPENGL+ cv2.WINDOW_AUTOSIZE)
cap = cv2.VideoCapture(0)
cap.set(3,1920)

# Data
robot_broadcast_data = {'states': {
                                   # 1: [(500, 500),    # middle of triangle base
                                   #     (520, 520)],   # point of traingle
                                   # 2: [(400, 400),    # middle of triangle base
                                   #     (420, 420)]    # point of traingle
                                   # etc...
                                   },
                        'balls': [], # List of Centroids
                        'settings': {'sight_range': 100,
                                     'dump_location': (20, 20)}
                        }

# Server
running = True

# Logging n stuff
logging.basicConfig(#filename='position_server.log',     # To a file. Or not.
                    filemode='w',                       # Start each run with a fresh log
                    format='%(asctime)s, %(levelname)s, %(message)s',
                    datefmt='%H:%M:%S',
                    level=logging.INFO, )              # Log info, and warning
n = 100             # Number of loops to wait for time calculation
t = time.time()


### Helper functions ###
def atan2_vec(vector):
    """
    Angle relative to horizontal axis. With vectors where the Y axis pointing down.
    A turn to the left is positive.
    :param vector:
    :return:
    """
    return -np.arctan2(vector[1], vector[0])


def vec_length(vector):
    return np.dot(vector, vector)**0.5


def pixel(img_grey, vector):
    if img_grey[vector[1], vector[0]]:
        return 1
    else:
        return 0


### Thread(s) ###

class SocketThread(Thread):
    def __init__(self):
        # Initialize server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        logging.info("Position server started on UDP {0}".format(SERVER_ADDR))
        Thread.__init__(self)

    def run(self):
        global robot_broadcast_data, running

        while running:
            data = pickle.dumps(robot_broadcast_data)

            try:
                sent = self.server_socket.sendto(data, SERVER_ADDR)
                # print(sent)
                time.sleep(0.025)
            except OSError as exc:
                if exc.errno == 55:
                    time.sleep(0.1)
                else:
                    raise
        self.server_socket.close()
        logging.info("Socket server stopped")



### Start it all up ###
socket_server = SocketThread()
socket_server.start()

while True:
    ok, img = cap.read()
    if not ok:
        continue    #and try again.
    img_height, img_width = img.shape[:2]
    # width = np.size(img, 1)

    # convert to grayscale
    img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # logging.debug("read greyscale image", t - time.time())

    # Simple adaptive mean thresholding
    values, img_grey = cv2.threshold(img_grey, THRESH, 255, cv2.ADAPTIVE_THRESH_MEAN_C)

    # Find contours and tree
    img_grey, contours, hierarchy = cv2.findContours(img_grey, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # logging.debug("found contours", t - time.time())

    # Uncomment to preview thresholded image
    #img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)

    robot_states = {}
    # Find triangular contours with at least 2 children. These must be our markers!
    for x in range(0, len(contours)):

        k = x
        c = 0

        # Look for nested triangles
        while (hierarchy[0][k][2] != -1):
            # As long as k has children [2], find that child and look for children again.
            # http://docs.opencv.org/3.1.0/d9/d8b/tutorial_py_contours_hierarchy.html
            k = hierarchy[0][k][2]
            c = c + 1

        if hierarchy[0][k][2] != -1:
            c = c + 1

        if c == 2:
            # To do: also check if it runs *exactly* 2 children deep. and not more.
            # This marker has at least two children. Now let's check if it's a triangle.
            approx = cv2.approxPolyDP(contours[x], cv2.arcLength(contours[x], True)*0.05, True)
            if len(approx) == 3:
                # We found a squarish object too.
                # Let it's corners be these vectors.
                a = approx[0][0]
                b = approx[1][0]
                c = approx[2][0]

                lengths = [vec_length(a-b), vec_length(b-c), vec_length(a-c)]
                shortest = min(lengths)
                shortest_idx = lengths.index(shortest)
                if shortest_idx == 0:
                    center = (a+b)/2
                    front = c
                elif shortest_idx == 1:
                    center = (c+b)/2
                    front = a
                else:   # shortest == 'ac':
                    center = (a+c)/2
                    front = b

                center = center.astype(int)
                heading = atan2_vec(front - center)

                # Now read code
                # Rotation matrix
                c = np.cos(heading)
                s = np.sin(heading)
                R = np.array([[-s, -c], [-c, s]])

                # Calculate the relative position of the code dots with some linear algebra.
                relative_code_positions = np.array([[0.375, 0.35],
                                                    [0.125, 0.35],
                                                    [-0.125, 0.35],
                                                    [-0.375, 0.35]])

                # Now do a dot product of the relative positions with the center position,
                # and offset this back to position of the robot to find matrix of absolute code pixel positions
                locations = (center + np.dot(relative_code_positions * shortest, R)).astype(int)

                # Now check all code pixels and do a binary addition
                robot_id = 0
                for i in range(4):
                    try:
                        p = img_grey[locations[i][1], locations[i][0]]
                    except: # The needed pixel is probably outside the image.
                        robot_id = -1
                        break
                    if not p:
                        robot_id += 2 ** i

                # Draw the data
                cv2.putText(img,
                            u"{0:.2f} rad, code: {1}, x:{2}, y:{3}".format(heading, robot_id, center[0], img_height - center[1]),
                            tuple(center),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 4)

                # Draw binary robot id marker positions
                for l in locations:
                    cv2.circle(img, tuple(l), 4, (0, 255, 0), -1)

                # Draw the contour of our triangle
                cv2.drawContours(img, [approx], -1, (0, 255, 0))

                # Save the data in our global dictionary
                robot_states[robot_id] = [(center[0], img_height - center[1]),  # Triangle Center with origin at bottom left
                                          (front[0], img_height - front[1])]    # Triangle Top with origin at bottom left

    robot_broadcast_data['states'] = robot_states
    # logging.debug("found markers", t - time.time())

    # Draw a + at the middle of the screen
    cv2.line(img, (img_width // 2 - 20, img_height // 2), (img_width // 2 + 20, img_height // 2), (0, 0, 255), 3)
    cv2.line(img, (img_width // 2, img_height // 2 - 20), (img_width // 2, img_height // 2 + 20), (0, 0, 255), 3)

    # Show all calculations in the preview window
    cv2.imshow("cam", img)
    # logging.debug("shown image", t - time.time())

    # Wait for the 'q' key. Dont use ctrl-c !!!
    keypress = cv2.waitKey(1) & 0xFF
    if keypress == ord('q'):
        break
    if n == 0:
        logging.info("Looptime: {0}, contours: {1}".format((time.time()-t)/100, len(contours)))
        n = 100
        t = time.time()
    else:
        n -= 1

### clean up ###
running = False
cap.release()
cv2.destroyAllWindows()
logging.info("Cleaned up")