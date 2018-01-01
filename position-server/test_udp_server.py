#!/usr/bin/env python3

import socket
import time
try:
    import cPickle as pickle
except:
    import pickle

# Create an UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
PORT = 50008

# Bind the socket to the broadcast port
server_address = ('255.255.255.255', PORT)

robot_broadcast_data = {'markers': {1: [(0, 0), (4, 3)],
                                    2: [(1919, 1079), (1919-4, 1079-3)],
                                    3: [(960, 540), (960+4, 540-3)]},
                        'balls': [],
                        'settings': {'sight_range': 300,
                                     'dump_location': (20, 20),
                                     'p_bot_midbase': (-4, -2),
                                     'p_bot_gripper': (0, 5),
                                     'field_height': 1080,
                                     'field_width': 1920,
                                     'cm_per_px': 0.1,
                                     'speed_per_cm_spring_extension': 0.5 ,
                                     'turnrate_per_cm_spring_extension': 5}
                        }


if __name__ == '__main__':
    while True:
        time.sleep(0.1)
        data = pickle.dumps(robot_broadcast_data)
        if data:
            try:
                sent = sock.sendto(data, server_address)
                time.sleep(0.5)
            except OSError as exc:
                if exc.errno == 55:
                    time.sleep(0.1)
                else:
                    raise
