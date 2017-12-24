#!/usr/bin/env python3

import socket
try:
    import cPickle as pickle
except:
    import pickle


### Initialize ###
PORT = 50008

### Create a socket ###
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', PORT))

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



