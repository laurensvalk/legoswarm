#!/usr/bin/env python3

import socket
from settings import robot_broadcast_data, SERVER_ADDR
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
SERVER_ADDR = ('255.255.255.255', PORT)

robot_broadcast_data['markers'] = {1: [(0, 0), (4, 3)],
                                    2: [(1919, 1079), (1919-4, 1079-3)],
                                    3: [(960, 540), (960+4, 540-3)]}

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
