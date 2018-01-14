import numpy as np
from robot_frames import transform_to_world_from_camera, transform_to_world_from_bot

def bounding_box(settings, midbase_marker, apex_marker):
    """Convert marker midbase and apex pixel location into bounding box, in pixels"""
    
    # Get transformation matrix from pixels to world frame
    H_to_world_from_camera = transform_to_world_from_camera(settings)    
    H_to_camera_from_world = H_to_world_from_camera.inverse()

    # Obtain transformation matrix between the robot and the world, for this robot
    H_to_world_from_bot = transform_to_world_from_bot(settings, 
                                                      H_to_world_from_camera*midbase_marker, # Midbase marker in world
                                                      H_to_world_from_camera*apex_marker) # Apex marker in world

    # Matrix of bounding box locations, in the robot frame
    bounding_box_in_robot = np.array(settings['bounding_box_cm']).T
    # Matrix of bounding box locations, in the world frame
    bounding_box_in_world = H_to_world_from_bot*bounding_box_in_robot
    # Matrix of bounding box locations, in camera pixels
    bounding_box_in_camera = H_to_camera_from_world*bounding_box_in_world
    # Revert the indexing so this can be seen as a list of coordinates
    return (bounding_box_in_camera.T).astype(int)

def preparse_robot_data(markers, balls, settings):

    # Initialize empty dictionary which we'll return
    localdata = {}

    # Determine who's who
    agents = markers.keys()

    # Get transformation matrix from pixels to world frame
    H_to_world_from_camera = transform_to_world_from_camera(settings)    

    # Obtain transformation matrix between the robot and the world, for each robot
    H_to_world_from_bot = {i: transform_to_world_from_bot(settings, 
                                                          H_to_world_from_camera*midbase_marker, # Midbase marker in world
                                                          H_to_world_from_camera*apex_marker) # Apex marker in world
                            for i, [midbase_marker, apex_marker] in markers.items()}

    # Empty dictionary of dictionaries of neighbor gripper locations
    neighborgrippers = {i: {} for i in agents}

    # Do the following computations for all agents, using the previously computed frame conversions
    for me in agents:

        # List of everyone except me
        other_agents = [i for i in agents if me != i]

        # Empty dictionary of neighboring gripper locations, in my frame of reference
        

        # Determine gripper location of everyone else in my reference frame
        for i in other_agents:
            # First, obtain the constant location a gripper in a robot frame
            p_bot_gripper = np.array(settings['p_bot_gripper'])

            # Transformation from another robot, to my reference frame
            H_to_me_from_otherbot = H_to_world_from_bot[me].inverse()@H_to_world_from_bot[i]

            # Gripper location of other robot, in my reference frame:
            p_me_othergripper = H_to_me_from_otherbot*p_bot_gripper

            # If that other gripper is in our field of view,
            # we consider it a neighbor and store the result
            if np.linalg.norm(p_me_othergripper) < settings['sight_range']:
                neighborgrippers[me][i] = p_me_othergripper


    localdata['neighborgrippers'] = neighborgrippers
    return localdata
