def read_broadcast():
    ##
    ## Dummy data
    ##
    robot_broadcast_data = {'markers':{1: [(500, 500), (520, 520)],   # midbase, apex
                                    2: [(400, 400), (420, 420)],   # midbase, apex
                                    3: [(200, 400), (320, 420)],   # midbase, apex
                                    4: [(500, 400), (520, 420)]    # midbase, apex
                                    },
                            'balls': [(23, 25),           # centroids of balls on camera
                                    (1800, 900)],
                            'settings': {'sight_range': 100,
                                        'dump_location': (20, 20),
                                        'p_b_midbase': (0.05, 0.05),
                                        'p_b_gripper': (0.00, 0.10),
                                        'dpx' : 1920,
                                        'dpy' : 1080,
                                        'eps' : 0.01
                                        }
                            }

    markers = robot_broadcast_data['markers']
    balls = robot_broadcast_data['balls']
    settings = robot_broadcast_data['settings']   
    return markers, balls, settings      