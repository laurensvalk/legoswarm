#!/usr/bin/env python3

import socket
import logging
import sys
import time
from threading import Thread
import gzip

try:
    import cPickle as pickle
except:
    import pickle

class CameraUDP(Thread):
    DECAY = 0.5  # seconds

    def __init__(self, port=50003):
        ### Initialize ###
        self.port = port
        self.robot_broadcast_data = {}
        self.running = True

        ### Create a socket ###
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(('', self.port))
        self.data_timestamp = 0
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(('', self.port))
        Thread.__init__(self)
        logging.debug("Inited thread")

    def stop(self):
        logging.debug("Stopping thread")
        self.running = False

    def get_data(self):
        if self.robot_broadcast_data and time.time() > self.data_timestamp + self.DECAY:
            self.robot_broadcast_data = {}
            # self.data_timestamp = time.time()
        return self.robot_broadcast_data

    ### Get robot positions from server ###
    def run(self):
        """
        Thread that writes UDP data into a global: robot_broadcast_data

        Usage:
        get_positions_thread = Thread(target=get_camera_data)
        get_positions_thread.start()

        to clean up:
        running=False
        :return:
        """
        logging.debug("Started thread")
        while self.running:
            data = self.read_from_socket()
            if data:
                self.robot_broadcast_data = data
                self.data_timestamp = time.time()
            time.sleep(0.07)
        self.s.close()

    def read_from_socket(self):
        try:
            data, server = self.s.recvfrom(1500)
            # if data:
            return pickle.loads(gzip.decompress(data))
        except:
            e = sys.exc_info()[0]
            logging.warning(e)
            return False
            # raise

if __name__ == '__main__':
    camera_thread = CameraUDP()
    camera_thread.start()
    while True:
        time.sleep(0.1)
        try:
            # Get robot positions from server
            print(camera_thread.get_data())
        except:
            # Time to panic, log some errors and kill others threads.
            camera_thread.stop()
            raise
    camera_thread.stop()
