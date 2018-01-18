robot_broadcast_data = {'markers': {#1: [(0, 0), (4, 3)],
                                    #2: [(1919, 1079), (1919-4, 1079-3)],
                                    #3: [(960, 540), (960+4, 540-3)]
                                    },
                        'localdata': {},
                        'balls': [],
                        'settings': {'sight_range': 300,
                                     'dump_location': (20, 20),
                                     'p_bot_midbase': (-2, -8),
                                     'p_bot_gripper': (0, 5),
                                     'field_height': 1080,
                                     'field_width': 1920,
                                     'cm_per_px': 0.1,
                                     'speed_per_cm_spring_extension': 0.2,
                                     'turnrate_per_cm_spring_extension': 0.8,
                                     'bounding_box_cm' : [
                                        # List of points in centimeters, encircling the robot
                                        # Starting at left wheel, then go counterclockwise
                                        [-8, 2.5], # Front end of left wheel
                                        [-8, -2.5], # Back end of left wheel
                                        [-5, -12], # Left rear wheel caster
                                        [5, -12], # right wheel caster
                                        [8, -2.5], # Back end of right wheel
                                        [8, 2.5], # Front end of right wheel                                        
                                        [5, 13], # Front-right end of gripper
                                        [-5, 13] # Front-left end of gripper
                                     ]
                                     }
                        }


SERVER_ADDR = ("255.255.255.255", 50008)
