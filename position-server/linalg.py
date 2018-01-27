import numpy as np


def atan2_vec(vector):
    """
    Angle relative to horizontal axis. With vectors where the Y axis pointing down.
    A turn to the left is positive.
    :param vector:
    :return:
    """
    return -np.arctan2(vector[1], vector[0])


def vec_length(vector):
    return np.dot(vector, vector)**0.5


def unit_vector(vector):
    return vector / vec_length(vector) + 0.000001