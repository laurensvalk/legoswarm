"""Several constant or frequently used composite transformations"""

from frametransformations import Transformation, ROW, COL
from numpy import array, identity, append
from numpy.linalg import norm

def transform_to_world_from_camera(settings):
    """Transform camera pixels into centimeters relative to camera midpoint"""
    # Transform to make positive y-axis point upwards
    rotation = array([[1, 0],[0, -1]]) # Flip y-axis
    translation = array([0, 0]) # No translation
    H_to_flipped_from_camera = Transformation(rotation, translation)

    # Transform to align axes with center of picture
    rotation = identity(2) # No rotation
    translation = array([-settings['field_width']/2, settings['field_height']/2])
    H_to_centered_from_flipped = Transformation(rotation, translation)

    # Scale to centimeters
    rotation = array([[settings['cm_per_px'], 0],[0, settings['cm_per_px']]]) # No rotation, but matrix plays role of scaling
    translation = array([0,0]) # No translation 
    H_to_world_from_centered = Transformation(rotation, translation)

    # Return the composite transformation
    return H_to_world_from_centered@H_to_centered_from_flipped@H_to_flipped_from_camera

def transform_to_world_from_robot(settings, p_world_midbase_marker, p_world_apexmarker):
    """Convert marker locations into transformation matrices"""

    # Constant transformation between label and robot
    H_to_bot_from_label = Transformation(identity(2), array(settings['p_bot_midbase']))
    H_to_label_from_bot = H_to_bot_from_label.inverse()


    # x frame axes of the label, expressed in the world: A line through the two markers
    label_xaxis_world = (p_world_midbase_marker-p_world_apexmarker)/norm(p_world_midbase_marker-p_world_apexmarker)

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
