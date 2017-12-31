import numpy as np

a = np.array([[1,0,0,], [0,-1,0], [0,0,1]])

b = np.array([[1,0,-960], [0,1,540], [0,0,1]])

c = np.identity(3) * 1.3

def transf_cam_to_world(point):
    point_t = np.array(list(point) + [1])
    result = c @ b @ a @ point_t
    return result[:2]

print(transf_cam_to_world((30,30)))

print(transf_cam_to_world([30,30]))

print(transf_cam_to_world(np.array([30,30])))
