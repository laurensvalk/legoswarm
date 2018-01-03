import numpy as np
from robot_frames import transform_to_world_from_camera, transform_to_world_from_bot

def preparse_robot_data(markers, balls, settings):

    # Initialize empty dictionary which we'll return
    localdata = {}

    # Determine who's who
    agents = [i for i in markers]

    # Empty dictionary of transformations from each robot to the world
    H_to_world_from_bot = {}

    # For every robot in the dataset, including me, determine position and orientation
    for i, [midbase_marker, apex_marker] in markers.items():
        # Get transformation matrix from pixels to world frame
        H_to_world_from_camera = transform_to_world_from_camera(settings)

        # Transform marker locations to world coordinates
        p_world_midbase_marker = H_to_world_from_camera*midbase_marker
        p_world_apex_marker = H_to_world_from_camera*apex_marker

        # Obtain transformation matrix between the robot and the world
        H_to_world_from_bot[i] = transform_to_world_from_bot(settings, p_world_midbase_marker, p_world_apex_marker)

    # Do the following computations for all agents, using the previously computed frame conversions
    for me in agents:
        # List of everyone except me
        other_agents = [i for i in agents if me != i]

        # Empty dictionary of neighboring gripper locations, in my frame of reference
        localdata['neighborgrippers'] = {}

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
                localdata['neighborgrippers'][i] = p_me_othergripper

    return localdata