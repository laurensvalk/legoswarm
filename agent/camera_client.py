#!/usr/bin/env python3

import socket
from threading import Thread
try:
    import cPickle as pickle
except:
    import pickle


### Initialize ###
PORT = 50008
robot_broadcast_data = {'states': {}, 'balls': {}, 'settings': {}}
running = True

### Create a socket ###
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', PORT))

### Get robot positions from server ###
def get_camera_data():
    """
    Thread that writes UDP data into a global: robot_broadcast_data

    Usage:
    get_positions_thread = Thread(target=get_camera_data)
    get_positions_thread.start()

    to clean up:
    running=False
    :return:
    """
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


if __name__ == '__main__':
    while 1:
        try:
            # Get robot positions from server
            data, server = s.recvfrom(2048)
            robot_broadcast_data = pickle.loads(data)
            print(robot_broadcast_data)
        except:
            # Time to panic, log some errors and kill others threads.
            raise



