import numpy as np
from robot_frames import transform_to_world_from_camera, transform_to_world_from_bot, transform_to_gripper_from_bot

def bounding_box(server_settings, midbase_marker, apex_marker):
    """Convert marker midbase and apex pixel location into bounding box, in pixels"""
    
    # Get transformation matrix from pixels to world frame
    H_to_world_from_camera = transform_to_world_from_camera(server_settings)    
    H_to_camera_from_world = H_to_world_from_camera.inverse()

    # Obtain transformation matrix between the robot and the world, for this robot
    H_to_world_from_bot = transform_to_world_from_bot(server_settings, 
                                                      H_to_world_from_camera*midbase_marker, # Midbase marker in world
                                                      H_to_world_from_camera*apex_marker) # Apex marker in world

    # Matrix of bounding box locations, in the robot frame
    bounding_box_in_robot = np.array(server_settings['bounding_box_cm']).T
    # Matrix of bounding box locations, in the world frame
    bounding_box_in_world = H_to_world_from_bot*bounding_box_in_robot
    # Matrix of bounding box locations, in camera pixels
    bounding_box_in_camera = H_to_camera_from_world*bounding_box_in_world
    # Revert the indexing so this can be seen as a list of coordinates
    return (bounding_box_in_camera.T).astype(int)

def get_ball_info(H_to_bot_from_world, ball_locations, server_settings):
    sorted_balls_relative_to_gripper = {}
    if len(ball_locations) > 0:
        # Get transformation matrix from pixels to world frame
        H_to_world_from_camera = transform_to_world_from_camera(server_settings)

        # Matrix of balls, in pixels
        balls_pixels = np.array(ball_locations).T

        # Matrix of balls, in world frame
        balls_world = H_to_world_from_camera*balls_pixels

        # Transformation from base frame to frame at gripper
        H_to_gripper_from_bot = transform_to_gripper_from_bot(server_settings)

        # Empty dictionary to fill during loop below

        for (agent, transformation) in H_to_bot_from_world.items():
            # Matrix of balls, transformed to the agent frame
            balls_relative_to_gripper = (H_to_gripper_from_bot@transformation)*balls_world

            # Scalar distance to the ball as seen from the current agent
            distances = np.linalg.norm(balls_relative_to_gripper, axis=0)
            # Sort by distance
            sorted_index = np.argsort(distances)
            n_balls_found = len(sorted_index)

            # Get only the closed balls to reduce data transmission:
            max_balls_send = server_settings['ball_info_max_size']

            if n_balls_found > max_balls_send:
                # If we see more balls than we can send, reduce data set
                sorted_index = sorted_index[0:max_balls_send]

            # Rebuild the array in order of distance
            sorted_balls_relative_to_gripper[agent] = [balls_relative_to_gripper[:,index].tolist() for index in sorted_index]

    # For each agent, return a sorted list of ball locations
    else:
        for (agent, transformation) in H_to_bot_from_world.items():
            sorted_balls_relative_to_gripper[agent] = []
    return sorted_balls_relative_to_gripper

def get_wall_info(H_to_bot_from_world, field_corners, server_settings):
    #               world x
    # 
    #            --------->
    #  A                          B
    #    +----------------------+
    #    |                      |    ^
    #    |                      |    |
    #    |                      |    |  world y
    #    |                      |    |
    #    |                      |    
    #    |                      |
    #    +----------------------+
    #  D                          C

    # Corner locations, in pixels
    corners_pixels = np.array(field_corners).T

    # Get transformation matrix from pixels to world frame
    H_to_world_from_camera = transform_to_world_from_camera(server_settings)        

    # Corner locations, in world
    corners_world = H_to_world_from_camera*corners_pixels

    # Gripper in agent frame
    my_gripper = np.array(server_settings['p_bot_gripper'])
    my_origin = np.array([0, 0])    

    # Empty dictionary to fill during loop below
    wall_info = {}
    for (agent, transformation) in H_to_bot_from_world.items():

        # Corners as seen by the agent
        corners_agent = transformation*corners_world

        # Extract the corner points
        A_agent, B_agent, C_agent, D_agent = corners_agent.T
        A_world, B_world, C_world, D_world = corners_world.T

        # Location of my gripper in the world frame
        my_gripper_world = transformation.inverse()*my_gripper

        # X and Y index
        X, Y = 0, 1

        # Distances
        distance_to_top = (A_world[Y]+B_world[Y])/2 - my_gripper_world[Y]
        distance_to_bottom = my_gripper_world[Y] - (C_world[Y]+D_world[Y])/2
        distance_to_left = my_gripper_world[X] - (A_world[X]+D_world[X])/2
        distance_to_right = (B_world[X]+C_world[X])/2 - my_gripper_world[X]

        # Distances as tuple
        distances = (distance_to_top, distance_to_bottom, distance_to_left, distance_to_right)

        # Lines e and f in agent frames, as unit vectors
        world_x = (B_agent - A_agent)/np.linalg.norm(B_agent-A_agent) 
        world_y = (B_agent - C_agent)/np.linalg.norm(B_agent-C_agent) 

        # Store information as dictionary for this agent
        wall_info[agent] = {'distances': distances,
                            'world_x' : world_x.tolist(),
                            'world_y': world_y.tolist(),
                            'corners': [A_agent.tolist(), B_agent.tolist(), C_agent.tolist(), D_agent.tolist()]}

    # For each agent, return a sorted list of ball locations
    return wall_info

def get_neighbor_info(markers, server_settings):
    # Determine who's who
    agents = markers.keys()

    # Get transformation matrix from pixels to world frame
    H_to_world_from_camera = transform_to_world_from_camera(server_settings)    

    # Obtain transformation matrix between the robot and the world, for each robot
    H_to_world_from_bot = {i: transform_to_world_from_bot(server_settings, 
                                                          H_to_world_from_camera*midbase_marker, # Midbase marker in world
                                                          H_to_world_from_camera*apex_marker) # Apex marker in world
                            for i, [midbase_marker, apex_marker] in markers.items()}

    # Precompute their inverses for later use
    H_to_bot_from_world = {i: H_to_world_from_bot[i].inverse() for i in agents}                        

    # Gripper in agent frame
    my_gripper = np.array(server_settings['p_bot_gripper'])
    my_origin = np.array([0, 0])

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
            neighbor_gripper = H_to_me_from_neighbor*my_gripper
            neighbor_info[me][neighbor]['gripper_location'] = neighbor_gripper.tolist()
            # Scalar distance to that gripper
            distance = np.linalg.norm(neighbor_gripper)

            # Also calculate center localtion
            neighbor_info[me][neighbor]['center_location'] = (H_to_me_from_neighbor*my_origin).tolist()
            
            # Check if that other gripper is in our "virtual" field of view
            neighbor_info[me][neighbor]['is_visible'] = True if distance < server_settings['sight_range'] else False

    # Return computed results
    return neighbor_info, H_to_bot_from_world


def make_data_for_robots(markers, ball_locations, field_corners, server_settings, robot_settings):

    # Information about the neighbors of each robot, in their own frame of reference
    neighbor_info, H_to_bot_from_world = get_neighbor_info(markers, server_settings)

    # Get the ball locations in each robot frame, sorted by distance from gripper
    ball_info = get_ball_info(H_to_bot_from_world, ball_locations, server_settings)

    # Get perpendicular lines to each wall in each robot frame of reference
    wall_info = get_wall_info(H_to_bot_from_world, field_corners, server_settings)

    result = {}
    for robot_id in markers:
        result[robot_id] = {'neighbors': neighbor_info[robot_id],
                            'balls': ball_info[robot_id],
                            'walls': wall_info[robot_id],
                            'robot_settings': robot_settings}
    return result
