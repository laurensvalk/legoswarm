#!/usr/bin/env python3

import socket
from settings import robot_broadcast_data, SERVER_ADDR
from parse_camera_data import preparse_robot_data, bounding_box
from numpy import array
import time
try:
    import cPickle as pickle
except:
    import pickle

# Create an UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

robot_broadcast_data['markers'] = {3: [(1171, 851), (1211, 812)],
                                    4: [(1547, 717), (1521, 673)],
                                    1: [(929, 510), (895, 555)],
                                    2: [(683, 380), (659, 332)]}

settings = robot_broadcast_data['settings']

# Run the debug for one example marker
midbase_marker, apex_marker = [(1171, 851), (1211, 812)] 
print(bounding_box(settings, midbase_marker, apex_marker))

robot_broadcast_data['localdata'] = preparse_robot_data(robot_broadcast_data['markers'],
                                                        robot_broadcast_data['balls'],
                                                        robot_broadcast_data['settings'])

if __name__ == '__main__':
    while True:
        time.sleep(0.1)
        data = pickle.dumps(robot_broadcast_data)
        if data:
            try:
                sent = sock.sendto(data, SERVER_ADDR)
                time.sleep(0.5)
            except OSError as exc:
                if exc.errno == 55:
                    time.sleep(0.1)
                else:
                    raise
