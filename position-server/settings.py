robot_settings = {
            'sight_range': 300,
            'dump_location': (0, 0),
            'p_bot_midbase': (-2, -8),
            'p_bot_gripper': (0, 11),
            'speed_per_unit_force': .75,
            'turnrate_per_unit_force': 2,
            'ball_close_enough': 12,
            'ball_grab_time': 3,  # s
            'max_balls_in_store': 5,
            'bounce_drive_speed': 4,
            'min_wall_distance': 7,
            'distance_to_purge_location': 10,
            'robot_avoidance_spring': [
            #                         c                     15 (pull when far)
            #                        / \
            #                      /     \
            #                    /         \ 
            #                  b             d------------   0 (no force at at 200 cm or beyond)
            #                /        
            #              / 
            #            /
            # ----------a                                    -20 (push when too close)
            # 
            #          0      30    160     200      
            # 
            #                                     
                [0,  -30], #a
                [30,  0 ], #b
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
                [160,  15],#c
                [200,  0 ]  #d
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
                [5 ,  10], #a
                [200,  0 ], #b                
            ],
            'spring_to_position' : [
                [0, 0],
                [5, 10]
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
    'sight_range' : robot_settings['sight_range'],
    'FILE' : '', #"test_images/error_scenario_too_many_balls.png"#""test_images/1516199702.jpg" #"test_images/test.jpg" # 1920 x 1080 afbeelding. png mag ook.
    'cm_per_ball_px': 0.13, # 263 cm diagonal = 1990 px, on the gounr
    'cm_per_marker_px': 0.13*0.96,#(1936-132)/1936, 
    'ball_info_max_size': 3, # Number of nearest balls each robot should get details of
    'depot_radius': 100,
    'bounding_box_cm': [
        # List of points in centimeters, encircling the robot
        # Starting at left wheel, then go counterclockwise
        [-12, 2.5], # Front end of left wheel
        [-12, -2.5], # Back end of left wheel
        [-8, -14], # Left rear wheel caster
        [8, -14], # right wheel caster
        [12, -2.5], # Back end of right wheel
        [12, 2.5], # Front end of right wheel
        [5, 19], # Front-right end of gripper
        [-5, 19] # Front-left end of gripper
    ]
}

# Allow Laurens to specify file to debug different scenarios without repeated git conflicts.
import platform
if 'Ubuntu' in platform.platform():
    print('Laurens Mode')
    server_settings['FILE'] = 'test_images/perspective/ball_perspective_case_3_nodump_solid_edge.png'
    server_settings['SERVER_BASE_PORT'] = 60000
