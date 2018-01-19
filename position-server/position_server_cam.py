#!/usr/bin/env python3

# Reads webcam data and outputs found triangle markers as an UDP broadcast
# Use q to stop this script, not ctrl-c!

import cv2
import numpy as np
import time
from threading import Thread
import socket
import logging
from platform import platform
from settings import robot_broadcast_data, SERVER_ADDR
from parse_camera_data import preparse_robot_data, bounding_box

try:
    import cPickle as pickle
except:
    import pickle


### Settings ###
THRESHOLD = 150        # Threshold for b/w version of camera image. Was 230 most of the time
WIDTH = 1920
HEIGHT = 1080
CROP_RECT = (0, 50, WIDTH - 50, HEIGHT - 80)
MIN_BALL_RADIUS_PX = 5
MAX_BALL_RADIUS_PX = 16
PLAYFIELD_COORDS = True
ROBOT_FOOTPRINT_CENTER_OFFSET = np.array([60, -30])
FILE = "test_images/1516199702.jpg" #"test_images/test.jpg" # 1920 x 1080 afbeelding. png mag ook.

### Initialize ###

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

def adjust_curve(image, factor=2.5):
    # build a lookup table mapping the pixel values [0, 255] to
    # their steepened curve values
    table = np.array([min(255, i*factor)
                      for i in np.arange(0, 256)]).astype("uint8")

    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)


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
            #print(robot_broadcast_data)
            try:
                sent = self.server_socket.sendto(data, SERVER_ADDR)
                # print(sent)
                time.sleep(0.025)
            except OSError as exc:
                if exc.errno == 55:
                    time.sleep(0.1)
                else:
                    raise
            time.sleep(0.3)
        self.server_socket.close()
        logging.info("Socket server stopped")



### Start it all up ###
socket_server = SocketThread()
socket_server.start()

# Detect playing field
while True:
    if not FILE:
        ok, img = cap.read()
        if not ok:
            continue    #and try again.
    else:
        img = cv2.imread(FILE)

    img_grey = cv2.Canny(img, 80, 180)
    img_grey = cv2.dilate(img_grey, np.ones((3,3)))
    img_grey, contours, hierarchy = cv2.findContours(img_grey, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)
    largest = 0
    for c in contours:
        approx = cv2.approxPolyDP(c, cv2.arcLength(c, True) * 0.02, True)
        if len(approx) == 4:
            area = cv2.contourArea(approx)
            if area > largest:
                playing_field = approx
                largest = area
                cv2.drawContours(img, [approx], -1, (255,0,255))
    cv2.drawContours(img, [playing_field], -1, (0, 255, 0), thickness=8)
    cv2.putText(img, "Press k if the playing field is detected", (150, 500), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 4)
    cv2.imshow("cam", img)

    # now that we have our screen contour, we need to determine
    # the top-left, top-right, bottom-right, and bottom-left
    # points so that we can later warp the image -- we'll start
    # by reshaping our contour to be our finals and initializing
    # our output rectangle in top-left, top-right, bottom-right,
    # and bottom-left order
    pts = playing_field.reshape(4, 2)
    rect = np.zeros((4, 2), dtype="float32")

    # the top-left point has the smallest sum whereas the
    # bottom-right has the largest sum
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    # compute the difference between the points -- the top-right
    # will have the minumum difference and the bottom-left will
    # have the maximum difference
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    # multiply the rectangle by the original ratio
    # rect *= ratio

    # now that we have our rectangle of points, let's compute
    # the width of our new image
    (tl, tr, br, bl) = rect
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
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")

    # calculate the perspective transform matrix and warp
    # the perspective to grab the screen
    M = cv2.getPerspectiveTransform(rect, dst)


    # Wait for the 'q' key. Dont use ctrl-c !!!
    keypress = cv2.waitKey(1) & 0xFF

    if keypress == ord('k'):
        break


while True:
    # time.sleep(2)
    if not FILE:
        ok, img = cap.read()
        if not ok:
            continue    #and try again.
    else:
        img = cv2.imread(FILE)
    # crop
    # img = np.array(img[50:1000, 0:1850])
    # img = np.array(img_cam)
    img = cv2.warpPerspective(img, M, (maxWidth, maxHeight))
    img_height, img_width = img.shape[:2]

    # convert to grayscale and adjust gamma curve
    # img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_grey = adjust_curve(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))

    # logging.debug("read greyscale image", t - time.time())

    # Simple adaptive mean thresholding
    values, img_grey = cv2.threshold(img_grey, THRESHOLD, 255, cv2.THRESH_BINARY)

    ## Alternative thresholding:
    # values, img_grey = cv2.threshold(img_grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # img_grey = cv2.adaptiveThreshold(img_grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)
    # img_grey = cv2.Canny(img_grey, 50, 150)
    # img_grey = cv2.dilate(img_grey, np.ones((3, 3)))


    # Find contours and tree
    img_grey, contours, hierarchy = cv2.findContours(img_grey, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # logging.debug("found contours", t - time.time())

    # Uncomment to preview thresholded image
    # img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)

    robot_markers = {}
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
                # We found a triangular object!
                # Let it's corners be these vectors.
                a = approx[0][0]
                b = approx[1][0]
                c = approx[2][0]

                lengths = [vec_length(a-b), vec_length(b-c), vec_length(a-c)]
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
                heading = atan2_vec(apex_marker - midbase_marker)

                # Rotation matrix for reading code
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
                            u"{0:.2f} rad, code: {1}, x:{2}, y:{3}".format(heading, robot_id, midbase_marker[0], midbase_marker[1]),
                            tuple(midbase_marker),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 4)

                # Draw binary robot id marker positions
                # for l in locations:
                #     cv2.circle(img, tuple(l), 4, (0, 255, 0), -1)

                # Draw the contour of our triangle
                cv2.drawContours(img, [approx], -1, (0, 255, 0))

                # Black out the shape of the robot in our source image
                box_center_offset = np.dot(ROBOT_FOOTPRINT_CENTER_OFFSET, R)
                blackout_region = (box_center_offset + midbase_marker, (165, 290), -heading / 3.1415 * 180)
                box = np.int0(cv2.boxPoints(blackout_region))
                cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
                cv2.fillConvexPoly(img_grey, box, 255)

                bb = bounding_box(robot_broadcast_data['settings'], midbase_marker, apex_marker)
                cv2.drawContours(img, [bb], 0, (0, 0, 255), 2)

                # Save the data in our global dictionary
                robot_markers[robot_id] = [(midbase_marker[0], midbase_marker[1]),  # Triangle Center with origin at bottom left
                                           (apex_marker[0], apex_marker[1])]    # Triangle Top with origin at bottom left


    # Found all robots, now let's detect balls.
    balls = []
    img_grey, contours, tree = cv2.findContours(img_grey, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (255, 0, 0))
    for c in contours:
        c, r = cv2.minEnclosingCircle(c)
        c = tuple(map(int, c))
        if 5 < r < 15:
            cv2.circle(img, c, int(r), (0, 255, 255), 2)
            balls += [c]

    robot_broadcast_data['balls'] = balls
    robot_broadcast_data['markers'] = robot_markers

    # Calculations to save time on client side
    robot_broadcast_data['localdata'] = preparse_robot_data(robot_broadcast_data['markers'], robot_broadcast_data['balls'], robot_broadcast_data['settings'])
    
    # logging.debug("found markers", t - time.time())

    # Draw a + at the middle of the screen
    # cv2.line(img, (img_width // 2 - 20, img_height // 2), (img_width // 2 + 20, img_height // 2), (0, 0, 255), 3)
    # cv2.line(img, (img_width // 2, img_height // 2 - 20), (img_width // 2, img_height // 2 + 20), (0, 0, 255), 3)

    # Show all calculations in the preview window
    cv2.imshow("cam", img)
    # logging.debug("shown image", t - time.time())

    # Wait for the 'q' key. Dont use ctrl-c !!!
    keypress = cv2.waitKey(1) & 0xFF

    if keypress == ord('q'):
        break
    if n == 0:
        logging.info("Looptime: {0}, contours: {1}".format((time.time()-t)/100, len(contours)))
        # cv2.imwrite("test_images/{0}.jpg".format(int(time.time())), img_cam)
        print(robot_broadcast_data)
        n = 100
        t = time.time()
    else:
        n -= 1
    # time.sleep(.3)

### clean up ###
running = False
if not FILE:
    cap.release()
cv2.destroyAllWindows()
logging.info("Cleaned up")
