settings = {
            'SERVER_ADDR': ("255.255.255.255", 50008),       
            'sight_range': 300,
            'dump_location': (20, 20),
            'p_bot_midbase': (-2, -8),
            'p_bot_gripper': (0, 5),
            'field_height': 1080,
            'field_width': 1920,
            'cm_per_px': 0.1,
            'speed_per_unit_force': 0.2,
            'turnrate_per_unit_force': 0.8,
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
            #                  b             d------------   0 (no force at at 10 cm or beyond 30)
            #                /        
            #              / 
            #            /
            # ----------a                                    -20 (push when too close)
            # 
            #          0      10     20      30      
            # 
            #                                     
                [0,  -20], #a
                [10,  0 ], #b
                [20,  15],#c
                [30,  0 ]  #d
            ]                                     
}

# The robots don't need all the settings. They need just these (add more if needed):
robot_keys = ['speed_per_unit_force', 'turnrate_per_unit_force', 'spring_between_robots']

# Create the settings dictionary with the above items
robot_settings = {key: settings[key] for key in robot_keys}