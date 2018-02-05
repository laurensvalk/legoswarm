import math

def atan2_vec(vector):
    """
    Angle relative to horizontal axis. With vectors where the Y axis pointing down.
    A turn to the left is positive.
    :param vector:
    :return:
    """
    return -math.atan2(vector[1], vector[0])


def vec_length(vector):
    return (vector[0]**2+vector[1]**2)**0.5

def unit_vector(vector):
    return vector / vec_length(vector) + 0.000001