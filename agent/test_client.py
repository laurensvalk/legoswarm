import logging, socket, time, gzip, pickle

MY_ID = 3

# Log settings
logging.basicConfig(format='%(asctime)s, %(levelname)s, %(message)s', datefmt='%H:%M:%S', level=logging.INFO)

# Start data thread
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 50000 + MY_ID
s.bind(('', 50000 + MY_ID))
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1500)
s.settimeout(0.2)
logging.debug("Listening on port {0}".format(port))

# States
FLOCKING = 'flocking'  # For now, just behavior that makes robots avoid one another
SEEK_BALL = 'seek ball'
STORE = 'store'
TO_DEPOT = 'to depot'
PURGE = 'purge'
LOW_VOLTAGE = 'low'
EXIT = 'exit'
BOUNCE = 'bounce'
DRIVE = 'drive'
PAUSE = 'pause'
TO_CENTER = 'to_center'

pause_end_time = time.time()
pause_next_state = SEEK_BALL

state = DRIVE
CHECK_VOLT_AFTER_LOOPS = 500
MAX_FAILS_BEFORE_WAIT = 8
loopcount = 0
failcount = 0
last_volt_check = time.time()

#################################################################
###### At every time step, read camera data, process it,
###### and steer robot accordingly
#################################################################

while True:

    logging.debug("Loop start")
    loopstart = time.time()
    loopcount += 1

    #################################################################
    ###### Receive data
    #################################################################

    try:
        # Get robot positions and settings from server
        compressed_data, server = s.recvfrom(1500)
        data = pickle.loads(gzip.decompress(compressed_data))

        # Get the data. Automatic exception if no data is available for MY_ID
        neighbor_info = data['neighbors']
        robot_settings = data['robot_settings']
        wall_info = data['walls']
        ball_info = data['balls']

        # Unpack some useful data from the information we received
        neighbors = neighbor_info.keys()

        # Check how many balls are near me
        number_of_balls = len(ball_info)

    except Exception as e:
        # Stop the loop if we're unable to get server data
        logging.warning("{0}: Reading data from port {1} failed. Waiting...".format(repr(e), port))
        if failcount > MAX_FAILS_BEFORE_WAIT:
            time.sleep(0.5)
        failcount += 1
        continue

    failcount = 0
    logging.debug("Got data after {0}ms".format(int((time.time() - loopstart) * 1000)))
    print(data)
    time.sleep(1)