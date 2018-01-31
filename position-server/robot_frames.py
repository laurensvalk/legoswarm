"""Several constant or frequently used composite transformations"""

from referenceframes.transformations import Transformation, ROW, COL
from numpy import array, identity, append
from numpy.linalg import norm

def transform_to_gripper_from_bot(server_settings):
    return Transformation(translation=-1*array(server_settings['p_bot_gripper']))

def transform_to_world_from_camera(server_settings, field_corners):
    """Transform camera pixels into centimeters relative to camera midpoint"""
    # Transform to make positive y-axis point upwards
    rotation = array([[1, 0],[0, -1]]) # Flip y-axis
    translation = array([0, 0]) # No translation
    H_to_flipped_from_camera = Transformation(rotation, translation)

    # Obtain previously computed field corners
    A, B, C, D = field_corners
    abs_offset = abs(server_settings['PLAYING_FIELD_OFFSET'])

    # Use field corners and border size to obtain cropped/warped image size
    image_width = abs(B[0]-A[0]) + 2*abs_offset
    image_height = abs(B[1]-C[1]) + 2*abs_offset

    # Transform to align axes with center of picture
    rotation = identity(2) # No rotation
    translation = array([-image_width/2, image_height/2])
    H_to_centered_from_flipped = Transformation(rotation, translation)

    # Scale to centimeters
    rotation = identity(2) # No rotation
    translation = array([0,0]) # No translation 
    scaling = server_settings['cm_per_px']
    H_to_world_from_centered = Transformation(rotation, translation, scaling)

    # Return the composite transformation
    return H_to_world_from_centered@H_to_centered_from_flipped@H_to_flipped_from_camera

def transform_to_world_from_bot(server_settings, p_world_midbase_marker, p_world_apex_marker):
    """Convert marker locations into transformation matrices"""

    # Constant transformation between label and robot
    H_to_bot_from_label = Transformation(identity(2), array(server_settings['p_bot_midbase']))
    H_to_label_from_bot = H_to_bot_from_label.inverse()

    # x frame axes of the label, expressed in the world: A line through the two markers
    label_xaxis_world = (p_world_apex_marker-p_world_midbase_marker)/norm(p_world_apex_marker-p_world_midbase_marker)

    # Ensure the axis is an actual column vector:
    label_xaxis_world = label_xaxis_world.reshape((2,1))

    # 90 degree rotation to obtain corresponding y-axis
    label_yaxis_world = array([[0, -1], [1, 0]])@label_xaxis_world

    # Transformation between world and label
    rotation = append(label_xaxis_world, label_yaxis_world, axis=COL)
    translation = p_world_midbase_marker
    H_to_world_from_label = Transformation(rotation, translation)    

    # Return the composite transformation
    return H_to_world_from_label@H_to_label_from_bot
