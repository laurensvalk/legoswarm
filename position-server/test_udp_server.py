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

# Data structure for robots with dummy data
robot_broadcast_data = {'states': {1: [(500, 500),    # middle of triangle base
                                       (520, 520)],   # point of traingle
                                   2: [(400, 400),    # middle of triangle base
                                       (420, 420)]    # point of traingle
                                   },
                        'balls': [(23, 24),           # centroids of balls on camera
                                  (1800, 900)],
                        'settings': {'sight_range': 100,
                                     'dump_location': (20, 20)}
                        }

if __name__ == '__main__':
    while True:
        time.sleep(0.033)
        data = pickle.dumps(robot_broadcast_data)
        if data:
            try:
                sent = sock.sendto(data, server_address)
                time.sleep(0.025)
            except OSError as exc:
                if exc.errno == 55:
                    time.sleep(0.1)
                else:
                    raise