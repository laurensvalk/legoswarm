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

def get_ball_info(H_to_bot_from_world, ball_locations, settings):

    # Get transformation matrix from pixels to world frame
    H_to_world_from_camera = transform_to_world_from_camera(settings)    

    # Matrix of balls, in pixels
    balls_pixels = np.array(ball_locations).T

    # Matrix of balls, in world frame
    balls_world = H_to_world_from_camera*balls_pixels

    # Empty dictionary to fill during loop below
    sorted_balls_in_agent_frames = {}
    for (agent, transformation) in H_to_bot_from_world.items():
        # Matrix of balls, transformed to the agent frame
        balls_in_agent_frame = transformation*balls_world

        # Scalar distance to the ball as seen from the current agent
        distances = np.linalg.norm(balls_in_agent_frame, axis=0)
        # Sort by distance
        sorted_index = np.argsort(distances)
        n_balls_found = len(sorted_index)

        # Get only the closed balls to reduce data transmission:
        max_balls_send = settings['ball_info_max_size']

        if n_balls_found > max_balls_send:
            # If we see more balls than we can send, reduce data set
            sorted_index = sorted_index[0:max_balls_send]

        # Rebuild the array in order of distance
        sorted_balls_in_agent_frames[agent] = [balls_in_agent_frame[:,index] for index in sorted_index]

    # For each agent, return a sorted list of ball locations
    return sorted_balls_in_agent_frames

def get_wall_info(H_to_bot_from_world, field_corners, settings):
    # In progress
    return 0

def get_neighbor_info(markers, settings):
    # Determine who's who
    agents = markers.keys()

    # Get transformation matrix from pixels to world frame
    H_to_world_from_camera = transform_to_world_from_camera(settings)    

    # Obtain transformation matrix between the robot and the world, for each robot
    H_to_world_from_bot = {i: transform_to_world_from_bot(settings, 
                                                          H_to_world_from_camera*midbase_marker, # Midbase marker in world
                                                          H_to_world_from_camera*apex_marker) # Apex marker in world
                            for i, [midbase_marker, apex_marker] in markers.items()}

    # Precompute their inverses for later use
    H_to_bot_from_world = {i: H_to_world_from_bot[i].inverse() for i in agents}                        

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
            H_to_me_from_neighbor = H_to_bot_from_world[me]@H_to_world_from_bot[neighbor]
            # Gripper location of other robot, in my reference frame:
            neighbor_info[me][neighbor]['gripper_location'] = H_to_me_from_neighbor*np.array(settings['p_bot_gripper'])
            # Scalar distance to that gripper
            distance = np.linalg.norm(neighbor_info[me][neighbor]['gripper_location'])
            # neighbor_info[me][neighbor]['gripper_distance'] = distance

            # Also calculate center localtion
            neighbor_info[me][neighbor]['center_location'] = H_to_me_from_neighbor * np.array([0, 0])
            # Can we do without this?
            # distance = np.linalg.norm(neighbor_info[me][neighbor]['center_location'])
            # neighbor_info[me][neighbor]['center_distance'] = distance

            # Check if that other gripper is in our "virtual" field of view
            neighbor_info[me][neighbor]['is_visible'] = True if distance < settings['sight_range'] else False

    # Return computed results
    return neighbor_info, H_to_bot_from_world


def make_data_for_robots(markers, ball_locations, field_corners, settings, robot_settings):

    # Information about the neighbors of each robot, in their own frame of reference
    neighbor_info, H_to_bot_from_world = get_neighbor_info(markers, settings)

    # Get the ball locations in each robot frame, sorted by distance from gripper
    ball_info = get_ball_info(H_to_bot_from_world, ball_locations, settings)
    if len(ball_info) > 5:
        ball_info = ball_info[:5]

    # Get perpendicular lines to each wall in each robot frame of reference
    wall_info = get_wall_info(H_to_bot_from_world, field_corners, settings)

    # Create data for returning
    return {'neighbor_info': neighbor_info,
            'ball_info': ball_info,
            'wall_info': wall_info,
            'robot_settings' : robot_settings}
