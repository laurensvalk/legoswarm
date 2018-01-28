settings = {
            'sight_range': 300,
            'dump_location': (20, 20),
            'p_bot_midbase': (-2, -8),
            'p_bot_gripper': (0, 5),
            'field_height': 1080,
            'field_width': 1920,
            'cm_per_px': 0.1,
            'speed_per_unit_force': 1,
            'turnrate_per_unit_force': 3,
            'ball_info_max_size' : 3, # Number of nearest balls each robot should get details of
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
            ],
            'spring_between_robots' : [
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
                [0,  -20], #a
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
                [0,  -10], #a
                [15,  0 ], #b
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
            ]                                      
}

# The robots don't need all the settings. They need just these (add more if needed):
robot_keys = ['speed_per_unit_force',
              'turnrate_per_unit_force',
              'spring_between_robots',
              'spring_to_walls',
              'spring_to_balls',
              'p_bot_gripper']

# Create the settings dictionary with the above items
robot_settings = {key: settings[key] for key in robot_keys}

# Server settings
SERVER_ADDR = ("255.255.255.255", 50008)
THRESHOLD = 150         # Threshold for b/w version of camera image. Was 230 most of the time
WIDTH = 1920            # Camera image
HEIGHT = 1080
PLAYING_FIELD_OFFSET = -50
MIN_BALL_RADIUS_PX = 5
MAX_BALL_RADIUS_PX = 16
MAX_AGENTS = 8
FILE = ''#"test_images/error_scenario_too_many_balls.png"#""test_images/1516199702.jpg" #"test_images/test.jpg" # 1920 x 1080 afbeelding. png mag ook.

# Allow Laurens to specify file to debug different scenarios without repeated git conflicts.
import platform
if 'Ubuntu' in platform.platform():
    print('Laurens Mode')
    FILE = 'test_images/test_ball_corners.png'
