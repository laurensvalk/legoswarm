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

def make_data_for_robots(markers, balls, settings, robot_settings):

    # Determine who's who
    agents = markers.keys()

    # Get transformation matrix from pixels to world frame
    H_to_world_from_camera = transform_to_world_from_camera(settings)    

    # Obtain transformation matrix between the robot and the world, for each robot
    H_to_world_from_bot = {i: transform_to_world_from_bot(settings, 
                                                          H_to_world_from_camera*midbase_marker, # Midbase marker in world
                                                          H_to_world_from_camera*apex_marker) # Apex marker in world
                            for i, [midbase_marker, apex_marker] in markers.items()}

    # Empty dictionaries that will get an entry for each agent in the loop that follows
    # For each robot, it contains information about its neighbors, from its own point of view
    neighbor_info = {}

    # Do the following computations for all agents, using the previously computed frame conversions
    for me in agents:

        # List of everyone except me
        neighbors = [i for i in agents if me != i]

        # I need an empty dictionary to store information about all my neighbors
        neighbor_info[me] = {neighbor : {} for neighbor in neighbors}

        # Determine gripper location and other properties of everyone else in my reference frame
        for neighbor in neighbors:
            # Transformation from another robot, to my reference frame
            H_to_me_from_neighbor = H_to_world_from_bot[me].inverse()@H_to_world_from_bot[neighbor]
            # Gripper location of other robot, in my reference frame:
            neighbor_info[me][neighbor]['gripper_location'] = H_to_me_from_neighbor*np.array(settings['p_bot_gripper'])
            # Scalar distance to that gripper
            distance = np.linalg.norm(neighbor_info[me][neighbor]['gripper_location'])
            neighbor_info[me][neighbor]['gripper_distance'] = distance
            # Check if that other gripper is in our "virtual" field of view
            neighbor_info[me][neighbor]['is_visible'] = True if distance < settings['sight_range'] else False

    # TODO: Ball locations & distances

    # Create data for returning
    return {'neighbor_info': neighbor_info,
            'robot_settings' : robot_settings}
