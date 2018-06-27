#!/usr/bin/env python3

# Reads webcam data and outputs found triangle markers as an UDP broadcast
# Use q to stop this script, not ctrl-c!

############################################################################
############################################################################
# Imports
############################################################################
############################################################################

import cv2
import numpy as np
import time
import socket
import logging
from threading import Thread
from platform import platform
import gzip

from antoncv import find_largest_rectangle_transform, offset_convex_polygon, rect_from_image_size, \
    find_nested_triangles, YELLOW, RED, PURPLE, GREEN, ORANGE, adjust_curve
from linalg import atan2_vec, vec_length

from importlib import reload
import settings # This is to make importlib/reload work.
from settings import server_settings, robot_settings
from parse_camera_data import make_data_for_robots, bounding_box

try:
    import cPickle as pickle
except:
    import pickle

############################################################################
############################################################################
# Initialize
############################################################################
############################################################################

# Initialize output window
if 'Ubuntu' in platform():
    # On Ubuntu without OpenGL
    cv2.namedWindow("cam")
else:
    # On Mac with OpenGL
    cv2.namedWindow("cam", cv2.WINDOW_OPENGL)        
if not server_settings['FILE']:
    cap = cv2.VideoCapture(0)
    cap.set(3, server_settings['WIDTH'])
    cap.set(4, server_settings['HEIGHT'])

# Data & robot settings
data_to_transmit = {}

# Server
running = True

# Logging
logging.basicConfig(#filename='position_server.log',     # To a file. Or not.
                    filemode='w',                        # Start each run with a fresh log
                    format='%(asctime)s, %(levelname)s, %(message)s',
                    datefmt='%H:%M:%S',
                    level=logging.INFO, )              # Log info, and warning


############################################################################
############################################################################
# UDP Server thread(s) ###
############################################################################
############################################################################

class SocketThread(Thread):
    def __init__(self):
        # Initialize server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1500)
        logging.info("Position broadcast started on UDP")
        Thread.__init__(self)

    def run(self):
        global data_to_transmit, running
        while running:
            # This is not very threadsafe, a lock would be better here. But hey, it works.
            for robot_id_key in data_to_transmit:
                port = server_settings['SERVER_BASE_PORT']+robot_id_key
                sent_bytes = self.udp_send_dict_key(data_to_transmit, robot_id_key, port)
                logging.debug("Sent {0} to port {1}".format(sent_bytes, port))
            time.sleep(0.06)
        self.server_socket.close()
        logging.info("Socket server stopped")

    def udp_send_dict_key(self, dictionary, key, port):
        if key in dictionary:
            data = gzip.compress(pickle.dumps(dictionary[key]))
            try:
                if data:
                    result = self.server_socket.sendto(data, ('255.255.255.255', port))
                    # print("Sent {0}b to port {1}".format(result, port))
                    return result
            except OSError as exc:
                if exc.errno == 55:
                    time.sleep(0.1)
                if exc.errno == 40:
                    print("Message for {1} too long: {0} bytes".format(len(data), key))
                else:
                    # pass
                    raise

### Start it all up ###
if __name__ == '__main__':

    socket_server = SocketThread()
    socket_server.start()

    ############################################################################
    ############################################################################
    # First detect playing field
    ############################################################################
    ############################################################################

    objects = 'edges'
    while True:
        if not server_settings['FILE']:
            ok, img = cap.read()
            img_cam = np.array(img)  # Duplicate for saving a situation to disk.
            if not ok:
                continue    #and try again.
        else:
            img = cv2.imread(server_settings['FILE'])

        img, M, dst, maxWidth, maxHeight = find_largest_rectangle_transform(img,
                                                                            server_settings['extra border outside'],
                                                                            look_for=objects)
        field_corners = offset_convex_polygon(dst,
                                              -server_settings['extra border outside'] + \
                                              server_settings['extra border inside'])

        cv2.imshow("cam", img)
        # Wait for the 'k' key. Dont use ctrl-c !!!
        keypress = cv2.waitKey(1000) & 0xFF

        if keypress == ord('y'):
            found_playing_field = True
            break
        elif keypress == ord('e'):
            objects = 'edges'
        elif keypress == ord('b'):
            objects = '4_blobs'
        elif keypress == ord('n'):
            # No Field edge detection, take the whole camera picture
            found_playing_field = False
            field_corners = rect_from_image_size(server_settings['WIDTH'], server_settings['HEIGHT'])
            break

    ############################################################################
    ############################################################################
    # Now we run the main image analysis loop, looking for balls and robots
    ############################################################################
    ############################################################################

    n = server_settings['reload_settings_after_n_loops']             # Number of loops to wait for time calculation
    t = time.time()     # Starttime for calculation
    while True:
        lt = time.time()
        logging.debug("Loop start: {0}".format(time.time()-lt))
        if not server_settings['FILE']:
            ok, img = cap.read()
            if not ok:
                continue  # and try again.
        else:
            img = cv2.imread(server_settings['FILE'])

        elapsed = time.time() - lt
        if elapsed > 0.1:
            logging.warning("{0}s for image. Slow camera! Bad cable? No OpenGL?".format(elapsed))
        else:
            logging.debug("Got image: {0}".format(elapsed))

        if found_playing_field:
            img = cv2.warpPerspective(img, M, (maxWidth, maxHeight))
            logging.debug("Image warped: {0}".format(time.time() - lt))

        robot_markers = {}
        img_grey, triangles = find_nested_triangles(img, threshold=server_settings['THRESHOLD'])
        logging.debug("Got triangles: {0}".format(time.time() - lt))

        # img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img_grey = adjust_curve(img_grey, 1.4)
        # values, img_grey = cv2.threshold(img_grey, server_settings['THRESHOLD'], 255, cv2.THRESH_BINARY)
        # img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)

        for triangle in triangles:
            # Let it's corners be these vectors.
            a = triangle[0][0]
            b = triangle[1][0]
            c = triangle[2][0]

            # Now lets find the middle of the base of the triangle and the apex.
            equal_sides = lengths = [vec_length(a - b), vec_length(b - c), vec_length(a - c)]
            shortest = min(lengths)
            shortest_idx = lengths.index(shortest)
            equal_sides.pop(shortest_idx)
            if min(equal_sides) * 1.09 < max(equal_sides):
                # If the equal sides are not so equal, skip this triangle...
                continue
            if shortest_idx == 0:
                midbase_marker = (a + b) / 2
                apex_marker = c
            elif shortest_idx == 1:
                midbase_marker = (c + b) / 2
                apex_marker = a
            else:   # shortest == 'ac':
                midbase_marker = (a + c) / 2
                apex_marker = b
            midbase_marker = midbase_marker.astype(int)

            # Find the direction in which the triangle is pointing
            heading = atan2_vec(apex_marker - midbase_marker)

            # Rotation matrix for reading code squares
            c = np.cos(heading)
            s = np.sin(heading)
            R = np.array([[-s, -c], [-c, s]])

            # Calculate the relative position of the code dots with some linear algebra.
            relative_code_positions = np.array([[0.4, 0.5],
                                                [0.125, 0.5],
                                                [-0.125, 0.5],
                                                [-0.4, 0.5]])

            # Do a dot product of the relative positions with the center position,
            # and offset this back to position of the robot to find matrix of absolute code pixel positions
            locations = (midbase_marker + np.dot(relative_code_positions * shortest, R)).astype(int)

            # Draw binary robot id marker positions
            for l in locations:
                cv2.circle(img, tuple(l), 4, (0, 255, 0), -1)

            # Now check all code pixels and do a binary addition
            robot_id = 0
            for i in range(4):
                try:
                    p = img_grey[locations[i][1], locations[i][0]]
                except:
                    # The needed pixel is probably outside the image.
                    robot_id = -1
                    break
                if not p:
                    robot_id += 2 ** i

            # Draw the data
            cv2.putText(img,
                        u"{0:.2f} rad, code: {1}, x:{2}, y:{3}".format(heading,
                                                                        robot_id,
                                                                        midbase_marker[0],
                                                                        midbase_marker[1]
                                                                        ),
                        tuple(midbase_marker),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, PURPLE, 4)

            # Draw the contour of our triangle
            cv2.drawContours(img, [triangle], -1, GREEN)

            # Black out the shape of the robot in our source image
            bb = bounding_box(server_settings, midbase_marker, apex_marker, field_corners)
            cv2.drawContours(img, [bb], 0, RED, 2)
            cv2.fillConvexPoly(img_grey, bb, 255)

            # Save the data in our dictionary
            robot_markers[robot_id] = [(midbase_marker[0], midbase_marker[1]),  # Triangle Center with origin at bottom left
                                        (apex_marker[0], apex_marker[1])]    # Triangle Top with origin at bottom left

        # Found all robots, now let's detect balls.
        balls = []

        if found_playing_field:
            # Get the size of the image
            img_height, img_width = img.shape[:2]

            # Erase the ball depot
            cv2.circle(img_grey, (img_width // 2, 0), server_settings['depot_radius'], (255,255,255), cv2.FILLED)

            mask = np.zeros((img_height, img_width), dtype=np.uint8)
            cv2.fillConvexPoly(mask, field_corners.astype(int), 255)
            cv2.bitwise_not(mask, dst=mask)
            cv2.bitwise_or(img_grey, mask, dst=img_grey)

        logging.debug("Masked field after: {0}s".format(time.time() - lt))
        # Now all robots & border are blacked out let's look for contours again.
        img_grey, contours, tree = cv2.findContours(img_grey, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            c, r = cv2.minEnclosingCircle(c)
            c = tuple(map(int, c))
            if server_settings['MIN_BALL_RADIUS_PX'] < r < server_settings['MAX_BALL_RADIUS_PX']:
                cv2.circle(img, c, int(r), YELLOW, 2)
                balls += [c]

        logging.debug("Listed balls after: {0}s".format(time.time() - lt))

        # Just define a random line for testing
        line = [(-200,-200), (200,200)]

        # Calculations to save time on client side
        data_to_transmit = make_data_for_robots(robot_markers,
                                                balls,
                                                field_corners,
                                                server_settings,
                                                robot_settings,
                                                line)

        logging.debug("Listed done calculations: {0}s".format(time.time() - lt))

        # Show all calculations in the preview window
        # img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(img, np.array([field_corners], dtype=int), -1, color=ORANGE, thickness=3)
        cv2.imshow("cam", img)

        # Wait for the 'q' key. Dont use ctrl-c !!!
        keypress = cv2.waitKey(1) & 0xFF

        if keypress == ord('q'):
            break
        elif keypress == ord('f'):
            robot_settings['state'] = 'flocking'
        elif keypress == ord('b'):
            robot_settings['state'] = 'drive'
        elif keypress == ord('s'):
            robot_settings['state'] = 'seek ball'
        elif keypress == ord('l'):
            robot_settings['state'] = 'straight line'
        elif keypress == ord(' '):
            # Save an image to disk:
            cv2.imwrite("test_images/{0}.jpg".format(int(time.time())), img_cam)
        else:
            robot_settings['state'] = ''
        if n == 0:
            logging.info("Looptime: {0}. Reloading settings.".format((time.time()-t)/server_settings['reload_settings_after_n_loops']))
            reload(settings)
            from settings import server_settings, robot_settings
            n = server_settings['reload_settings_after_n_loops']
            t = time.time()
        else:
            n -= 1

        # Don't run so often while debugging
        if 'Ubuntu' in platform():
            time.sleep(2)   

    # User has hit q. Time to clean up.
    running = False
    if not server_settings['FILE']:
        cap.release()
    cv2.destroyAllWindows()
    logging.info("Cleaned up")
