from numpy import array

# robot_broadcast_data = {'markers': {#1: [(0, 0), (4, 3)],
#                                     #2: [(1919, 1079), (1919-4, 1079-3)],
#                                     #3: [(960, 540), (960+4, 540-3)]
#                                     },
#                         'localdata': {},
#                         'balls': [],
#                         'settings': {'sight_range': 300,
#                                      'dump_location': (20, 20),
#                                      'p_bot_midbase': (-4, -2),
#                                      'p_bot_gripper': (0, 5),
#                                      'field_height': 1080,
#                                      'field_width': 1920,
#                                      'cm_per_px': 0.1,
#                                      'speed_per_cm_spring_extension': 0.1,
#                                      'turnrate_per_cm_spring_extension': 0.5}
#                         }

robot_broadcast_data = {'markers': {3: [(1171, 851), (1211, 812)],
                                    4: [(1547, 717), (1521, 673)],
                                    1: [(929, 510), (895, 555)],
                                    2: [(683, 380), (659, 332)]},
                        'localdata': {
                            'neighborgrippers': {3: array([-60.06649611, -26.26536281]),
                                                 4: array([-69.28128691, -56.94457068]),
                                                 1: array([-34.67721619, -17.713376  ])}
                                    },
                        'balls': [],
                        'settings': {'sight_range': 300,
                                     'dump_location': (20, 20),
                                     'p_bot_midbase': (-4, -2),
                                     'p_bot_gripper': (0, 5),
                                     'field_height': 1080,
                                     'field_width': 1920,
                                     'cm_per_px': 0.1,
                                     'speed_per_cm_spring_extension': 0.1,
                                     'turnrate_per_cm_spring_extension': 0.5}}

SERVER_ADDR = ("255.255.255.255", 50008)
