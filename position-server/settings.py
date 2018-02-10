robot_settings = {
            'sight_range': 300,
            'dump_location': (0, 0),
            'p_bot_midbase': (-2, -8),
            'p_bot_gripper': (0, 11),
            'p_bot_rear': (0, -15),
            'speed_per_unit_force': .75,
            'turnrate_per_unit_force': 2,
            'ball_close_enough': 18,
            'ball_grab_time': 3,  # s
            'max_balls_in_store': 5,
            'bounce_drive_speed': 4,
            'min_wall_distance': 7,
            'distance_to_purge_location': 11,
            'robot_avoidance_spring': [
            # 0                  b ------------   0 (no force at at 30 cm or beyond)
            #                  /
            #                /
            #              /
            # -20 ---------a                     -20 (push when too close)
            # 
            #          0      30    160     200      
            # 
            #                                     
                [0,  -30], #a
                [40,  0 ], #b
            ],
            'robot_avoidance_spring_inferior': [
            # 0                  b ------------   0 (no force at at 30 cm or beyond)
            #                  /
            #                /
            #              /
            # -20 ---------a                     -20 (push when too close)
            #
            #          0      30    160     200
            #
            #
                [0,  -30], #a
                [20,  0 ], #b
            ],
            'robot_attraction_spring' : [
            #                         c                     15 (pull when far)
            #                        / \
            #                      /     \
            #                    /         \
            # -------- a -------b             d------------   0 (no force at at 200 cm or beyond)
            #
            #          0      30    160     200
            #
            #
                [0,  0], #a
                [30,  0 ], #b
                [60,  20],#c
                [200,  20 ]  #d
            ],
            'spring_to_walls' : [
            #
            #                  b ----------   0 (no force at at 10 cm or beyond)
            #                /        
            #              / 
            #            /
            # ----------a                    -20 (push when too close)
            # 
            #          0      30      
            # 
            #                                     
            #     [-30, 30],
                [0,  -20], #a
                [25, 0 ], #b
            ],
            'short_spring_to_walls' : [
            #
            #                  b ----------   0 (no force at at 10 cm or beyond)
            #                /
            #              /
            #            /
            # ----------a                    -20 (push when too close)
            #
            #          0      30
            #
            #
            #     [-30, 30],
                [0,  -20], #a
                [12, 0 ], #b
            ],
            'spring_to_balls' : [
            #
            # -------a                        10  (Pull when nearer than 5)          
            #         \
            #          \
            #           \
            #            b-----------         0   (no force at at 10 cm or beyond)
            # 
            #        5   10      
            # 
            #                                     
                [0 ,  0], #a
                [10,  10 ], #b
            ],
            'spring_to_depot' : [
                [0 ,  20], #a
                [10,  30 ], #b
                [50, 15]
            ],
            'spring_to_position' : [
                [0, 0],
                [10, 20]
            ]

}

# Server settings
server_settings = {
    'SERVER_BASE_PORT' : 50000,
    'THRESHOLD' : 150,         # Threshold for b/w version of camera image. Was 230 most of the time
    'WIDTH' : 1920,            # Camera image
    'HEIGHT' : 1080,
    'PLAYING_FIELD_OFFSET' : -50,
    'MIN_BALL_RADIUS_PX' : 5,
    'MAX_BALL_RADIUS_PX' : 16,
    'MAX_AGENTS' : 8,
    'p_bot_midbase' : robot_settings['p_bot_midbase'],
    'p_bot_gripper' : robot_settings['p_bot_gripper'],
    'p_bot_rear' : robot_settings['p_bot_rear'],
    'sight_range' : robot_settings['sight_range'],
    'FILE' : '',#'"test_images/error_scenario_too_many_balls.png",#""test_images/1516199702.jpg" #"test_images/test.jpg" # 1920 x 1080 afbeelding. png mag ook.
    'cm_per_ball_px': 0.13, # 263 cm diagonal = 1990 px, on the gound
    'cm_per_marker_px': 0.13*(1936-132*0.8)/1936, # Markers are at approx 13.2 cm from ground
    'cm_per_bounding_px': 0.13*(1936-60)/1936, # dimensions relevant for bounding box are at approx 6cm above ground
    'ball_info_max_size': 3, # Number of nearest balls each robot should get details of
    'depot_radius': 200, #pixels
    'bounding_box_cm': [
        # List of points in centimeters, encircling the robot
        # Starting at left wheel, then go counterclockwise
        [-13, 2.5], # Front end of left wheel
        [-13, -2.5], # Back end of left wheel
        [-8, -14], # Left rear wheel caster
        [0, -17], # Motor cable
        [8, -14], # right wheel caster
        [13, -2.5], # Back end of right wheel
        [13, 2.5], # Front end of right wheel
        [5, 20], # Front-right end of gripper
        [-5, 20] # Front-left end of gripper
    ]
}

# Allow Laurens to specify file to debug different scenarios without repeated git conflicts.
import platform
if 'Ubuntu' in platform.platform():
    print('Laurens Mode')
    server_settings['FILE'] = 'test_images/over_the_edge_error_nodump.jpg'
    server_settings['SERVER_BASE_PORT'] = 60000
