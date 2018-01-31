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

from antoncv import find_largest_n_side, sorted_rect, offset_convex_polygon, rect_from_image_size, \
    find_nested_triangles, YELLOW, RED, PURPLE, GREEN, ORANGE
from linalg import atan2_vec, vec_length

from importlib import reload
import settings # This is to make importlib/reload work.
from settings import all_settings, robot_settings, WIDTH, HEIGHT, FILE, PLAYING_FIELD_OFFSET
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
if not FILE:
    cap = cv2.VideoCapture(0)
    cap.set(3, WIDTH)
    cap.set(4, HEIGHT)

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
            for robot_id_key in data_to_transmit:
                sent_bytes = self.udp_send_dict_key(data_to_transmit, robot_id_key, 50000+robot_id_key)
                # print(sent_bytes)
            time.sleep(0.07)
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

    while True:
        if not FILE:
            ok, img = cap.read()
            if not ok:
                continue    #and try again.
        else:
            img = cv2.imread(FILE)

        img_grey, largest_rect = find_largest_n_side(img, sides=4)

        # Optionally review edge-finding image
        img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)

        # Now that we have our playing field contour, we need to determine
        # the top-left, top-right, bottom-right, and bottom-left
        # points so that we can later warp the image. So we sort the polygon points in this order
        rect = sorted_rect(largest_rect)
        offset_rect = offset_convex_polygon(rect, PLAYING_FIELD_OFFSET)

        # Present the found rectangles to the user
        cv2.drawContours(img, [largest_rect.astype(int)], -1, GREEN, thickness=8)
        cv2.drawContours(img, [offset_rect.astype(int)], -1, PURPLE, thickness=8)
        cv2.putText(img, "Press y if the playing field is detected, press n if not", (100, 500),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, GREEN, 4)

        cv2.imshow("cam", img)
        # Wait for the 'k' key. Dont use ctrl-c !!!
        keypress = cv2.waitKey(1000) & 0xFF

        if keypress == ord('y'):
            # now that we have our rectangle of points, let's compute
            # the width of our new image
            (tl, tr, br, bl) = offset_rect
            widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
            widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))

            # ...and now for the height of our new image
            heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
            heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))

            # take the maximum of the width and height values to reach
            # our final dimensions
            maxWidth = max(int(widthA), int(widthB))
            maxHeight = max(int(heightA), int(heightB))

            # construct our destination points which will be used to
            # map the screen to a top-down, "birds eye" view
            dst = rect_from_image_size(maxWidth - 1, maxHeight - 1)

            # calculate the perspective transform matrix and warp
            # the perspective to grab the screen
            M = cv2.getPerspectiveTransform(offset_rect, dst)

            found_playing_field = True
            field_corners = offset_convex_polygon(dst, -PLAYING_FIELD_OFFSET)
            # field_corners = rect
            break

        elif keypress == ord('n'):
            found_playing_field = False
            field_corners = rect_from_image_size(WIDTH, HEIGHT)
            break

    ############################################################################
    ############################################################################
    # Now we run the main image analysis loop, looking for balls and robots
    ############################################################################
    ############################################################################

    n = 100             # Number of loops to wait for time calculation
    t = time.time()     # Starttime for calculation
    while True:
        lt = time.time()
        logging.debug("Loop start: {0}".format(time.time()-lt))
        if not FILE:
            ok, img = cap.read()
            if not ok:
                continue  # and try again.
        else:
            img = cv2.imread(FILE)

        elapsed = time.time() - lt
        if elapsed > 0.1:
            logging.warning("{0}s for image. Slow camera! Bad cable? No OpenGL?".format(elapsed))
        else:
            logging.debug("Got image: {0}".format(elapsed))

        if found_playing_field:
            img = cv2.warpPerspective(img, M, (maxWidth, maxHeight))
            logging.debug("Image warped: {0}".format(time.time() - lt))

        robot_markers = {}
        img_grey, triangles = find_nested_triangles(img)
        logging.debug("Got triangles: {0}".format(time.time() - lt))

        img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)

        for triangle in triangles:
            # Let it's corners be these vectors.
            a = triangle[0][0]
            b = triangle[1][0]
            c = triangle[2][0]

            # Now lets find the middle of the base of the triangle and the apex.
            lengths = [vec_length(a - b), vec_length(b - c), vec_length(a - c)]
            shortest = min(lengths)
            shortest_idx = lengths.index(shortest)
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
            relative_code_positions = np.array([[0.375, 0.5],
                                                [0.125, 0.5],
                                                [-0.125, 0.5],
                                                [-0.375, 0.5]])

            # Do a dot product of the relative positions with the center position,
            # and offset this back to position of the robot to find matrix of absolute code pixel positions
            locations = (midbase_marker + np.dot(relative_code_positions * shortest, R)).astype(int)

            # Draw binary robot id marker positions
            # for l in locations:
            #     cv2.circle(img, tuple(l), 4, (0, 255, 0), -1)

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
            bb = bounding_box(all_settings, midbase_marker, apex_marker)
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

            # Erase the borders
            cv2.polylines(img_grey,
                            np.array([rect_from_image_size(img_width, img_height)], dtype=int),
                            True,
                            255,
                            thickness=abs(PLAYING_FIELD_OFFSET)*2+8)
            # mask = np.zeros((img_height, img_width), dtype=np.uint8)
            # cv2.fillConvexPoly(mask, field_corners.astype(int), 255)
            # cv2.bitwise_not(mask, dst=mask)
            # cv2.bitwise_or(img_grey, mask, dst=img_grey)

        # Now all robots & border are blacked out let's look for contours again.
        img_grey, contours, tree = cv2.findContours(img_grey, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            c, r = cv2.minEnclosingCircle(c)
            c = tuple(map(int, c))
            if 5 < r < 15:
                cv2.circle(img, c, int(r), YELLOW, 2)
                balls += [c]

        # Calculations to save time on client side
        data_to_transmit = make_data_for_robots(robot_markers, balls, field_corners, all_settings, robot_settings)

        # Show all calculations in the preview window
        # img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(img, np.array([field_corners], dtype=int), -1, color=ORANGE, thickness=3)
        cv2.imshow("cam", img)

        # Wait for the 'q' key. Dont use ctrl-c !!!
        keypress = cv2.waitKey(1) & 0xFF

        if keypress == ord('q'):
            break
        if n == 0:
            logging.info("Looptime: {0}. Reloading settings.".format((time.time()-t)/100))
            reload(settings)
            from settings import all_settings, robot_settings, WIDTH, HEIGHT, FILE, PLAYING_FIELD_OFFSET
            # Uncomment to save an image to disk:
            # cv2.imwrite("test_images/{0}.jpg".format(int(time.time())), img_cam)
            n = 100
            t = time.time()
        else:
            n -= 1

        # Don't run so often while debugging
        if 'Ubuntu' in platform():
            time.sleep(2)   

    # User has hit q. Time to clean up.
    running = False
    if not FILE:
        cap.release()
    cv2.destroyAllWindows()
    logging.info("Cleaned up")
