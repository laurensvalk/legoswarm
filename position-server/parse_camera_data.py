import numpy as np
from robot_frames import transform_to_world_from_marker_pixels, transform_to_world_from_ball_pixels, transform_to_world_from_bot, transform_to_gripper_from_bot, transform_to_world_from_bounding_pixels

def bounding_box(server_settings, midbase_marker, apex_marker, field_corners):
    """Convert marker midbase and apex pixel location into bounding box, in pixels"""
    
    # Get transformation matrix from pixels to world frame
    H_to_world_from_marker_pixels = transform_to_world_from_marker_pixels(server_settings, field_corners)
    
    # Obtain transformation matrix between the robot and the world, for this robot
    H_to_world_from_bot = transform_to_world_from_bot(server_settings, 
                                                      H_to_world_from_marker_pixels*midbase_marker, # Midbase marker in world
                                                      H_to_world_from_marker_pixels*apex_marker) # Apex marker in world

    # Matrix of bounding box locations, in the robot frame
    bounding_box_in_robot = np.array(server_settings['bounding_box_cm']).T
    # Matrix of bounding box locations, in the world frame
    bounding_box_in_world = H_to_world_from_bot*bounding_box_in_robot
    # Matrix of bounding box locations, in pixels concerning bounding box
    H_to_world_from_bounding_pixels = transform_to_world_from_bounding_pixels(server_settings, field_corners)
    bounding_box_in_bounding_pixels = H_to_world_from_bounding_pixels.inverse()*bounding_box_in_world
    # Revert the indexing so this can be seen as a list of coordinates
    return (bounding_box_in_bounding_pixels.T).astype(int)

def get_depot_info(H_to_bot_from_world, server_settings, sort_by_distance=False):
    # Load the absolute depot locations
    depot_locations_world = np.array(server_settings['depots_world']).T

    # Empty dictionary which we'll fill for each agent in this loop
    depots_agents = {}
    for (agent, transformation) in H_to_bot_from_world.items():
        # Transform the gripper to current agent
        depots_agent_frame = transformation*depot_locations_world

        if not sort_by_distance:
            # If no sorting is needed, we're already done
            depots_agents[agent] = depots_agent_frame.T.tolist()
        else:
            # Scalar distance to the depot as seen from the current agent
            distances = np.linalg.norm(depots_agent_frame, axis=0)

            # Sort by distance
            sorted_index = np.argsort(distances)
            n_depots_found = len(sorted_index)

            # Rebuild the array in order of distance
            depots_agents[agent] = [depots_agent_frame[:,index].tolist() for index in sorted_index]
            
    return depots_agents

def get_ball_info(H_to_bot_from_world, ball_locations, server_settings, field_corners):
    sorted_balls_relative_to_gripper = {}
    if len(ball_locations) > 0:
        # Get transformation matrix from pixels to world frame
        H_to_world_from_ball_pixels = transform_to_world_from_ball_pixels(server_settings, field_corners)

        # Matrix of balls, in pixels
        balls_pixels = np.array(ball_locations).T

        # Matrix of balls, in world frame
        balls_world = H_to_world_from_ball_pixels*balls_pixels

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

            # Balls DEBUG. Leave this here for now so I can uncomment if needed during debugging
            # if agent == 3:
            #     abs_balls = [balls_world[:,index] for index in sorted_index]
            #     gripper_balls = [balls_relative_to_gripper[:,index] for index in sorted_index]
            #     print("----")
            #     print(abs_balls)
            #     print(gripper_balls)
            #     pass

    # For each agent, return a sorted list of ball locations
    else:
        for (agent, transformation) in H_to_bot_from_world.items():
            sorted_balls_relative_to_gripper[agent] = []
    return sorted_balls_relative_to_gripper

def get_wall_info(H_to_bot_from_world, server_settings, field_corners):
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
    H_to_world_from_marker_pixels = transform_to_world_from_marker_pixels(server_settings, field_corners)        

    # Corner locations, in world
    corners_world = H_to_world_from_marker_pixels*corners_pixels

    # Gripper in agent frame
    my_gripper = np.array(server_settings['p_bot_gripper'])
    my_rear = np.array(server_settings['p_bot_rear'])
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
        my_rear_world = transformation.inverse()*my_rear

        # X and Y index
        X, Y = 0, 1

        # Check if gripper or rear is closer to the walls
        closest_to_top = max(my_gripper_world[Y], my_rear_world[Y])
        closest_to_bottom = min(my_gripper_world[Y], my_rear_world[Y])
        closest_to_left = min(my_gripper_world[X], my_rear_world[X])
        closest_to_right = max(my_gripper_world[X], my_rear_world[X])

        # Distances
        distance_to_top = (A_world[Y]+B_world[Y])/2 - closest_to_top
        distance_to_bottom = closest_to_bottom - (C_world[Y]+D_world[Y])/2
        distance_to_left = closest_to_left - (A_world[X]+D_world[X])/2
        distance_to_right = (B_world[X]+C_world[X])/2 - closest_to_right

        # Distances as tuple
        micron = 0.001
        distances = (max(distance_to_top, micron), max(distance_to_bottom, micron), max(distance_to_left, micron), max(distance_to_right, micron))

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


def line_coefs(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False

def get_line_info(H_to_bot_from_world, server_settings, line):
    # Load the absolute depot locations
    line_points_world = np.array(line).T

    # Gripper in agent frame
    my_gripper = np.array(server_settings['p_bot_gripper'])

    # Empty dictionary which we'll fill for each agent in this loop
    line_info = {}
    for (agent, transformation) in H_to_bot_from_world.items():
        # Transform the gripper to current agent
        endpoint_agent_frame = transformation * line_points_world[1]

        # Location of my gripper in the world frame
        my_gripper_world = transformation.inverse() * my_gripper

        L1 = line_coefs(line[0], line[1])
        line_vec = (line_points_world[1] - line_points_world[0])
        line_vec_perp = np.array(-line_vec[1], line_vec[0])
        line_vec_perp_from_gripper = my_gripper_world + line_vec_perp

        L2 = line_coefs(my_gripper_world, line_vec_perp_from_gripper)
        closest_point = np.array(intersection(L1,L2))
        closest_point_agent_frame = transformation * closest_point

        line_info[agent] = {'endpoint': endpoint_agent_frame, 'closest_point': closest_point_agent_frame}

    return line_info


def get_neighbor_info(markers, server_settings, field_corners):
    # Determine who's who
    agents = markers.keys()

    # Get transformation matrix from pixels to world frame
    H_to_world_from_marker_pixels = transform_to_world_from_marker_pixels(server_settings, field_corners)    

    # Obtain transformation matrix between the robot and the world, for each robot
    H_to_world_from_bot = {i: transform_to_world_from_bot(server_settings, 
                                                          H_to_world_from_marker_pixels*midbase_marker, # Midbase marker in world
                                                          H_to_world_from_marker_pixels*apex_marker) # Apex marker in world
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


def make_data_for_robots(markers, ball_locations, field_corners, server_settings, robot_settings, line):

    # Information about the neighbors of each robot, in their own frame of reference
    neighbor_info, H_to_bot_from_world = get_neighbor_info(markers, server_settings, field_corners)

    # Get the ball locations in each robot frame, sorted by distance from gripper
    ball_info = get_ball_info(H_to_bot_from_world, ball_locations, server_settings, field_corners)

    # # Get depot locations
    # left, top = field_corners[0]
    # right, bottom = field_corners[2]
    # width = right - left
    # height = bottom - top
    # depots = [(x * width + left, y * height + top) for x,y in server_settings['depots']]
    # depot_info = get_ball_info(H_to_bot_from_world, depots, server_settings, field_corners)

    # Get depot locations in each of the robot frames (relative to robot base)
    depot_info = get_depot_info(H_to_bot_from_world, server_settings)

    # Get perpendicular lines to each wall in each robot frame of reference
    wall_info = get_wall_info(H_to_bot_from_world, server_settings, field_corners)

    line_info = get_line_info(H_to_bot_from_world, server_settings, line)

    result = {}
    for robot_id in markers:
        result[robot_id] = {'neighbors': neighbor_info[robot_id],
                            'balls': ball_info[robot_id],
                            'walls': wall_info[robot_id],
                            'depots': depot_info[robot_id],
                            'robot_settings': robot_settings}
    return result
