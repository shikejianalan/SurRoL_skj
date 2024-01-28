import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dvrk import mtm

def forward_point_on_trajectory(p, trajectory, forward_steps=5):
    """
    Find a forward point on the trajectory from the nearest point to p.    
    :param p: numpy array, the position vector of the current point (x, y)
    :param trajectory: numpy array of shape (n, 2), the trajectory points (x, y)
    :param forward_steps: int, the number of steps to move forward along the trajectory
    :return: numpy array, the position vector of the forward point (x, y)
    """
    # Find the nearest point index on the trajectory
    distances = np.linalg.norm(trajectory - p, axis=1)
    nearest_index = np.argmin(distances)    # Calculate the forward index, ensuring it doesn't exceed the trajectory length
    forward_index = min(nearest_index + forward_steps, len(trajectory) - 1)   
    return np.min(distances), trajectory[forward_index]

def trajectory_forward_field_3d(trajectory, position, k_att, forward_steps=5):
    """
    Plot the 3D potential field with arrows pointing towards a forward point on the trajectory.    
    :param trajectory: numpy array of shape (n, 2), the trajectory points (x, y)
    :param k_att: float, scaling factor for the attractive potential
    :param grid_size: int, the size of the grid for visualization
    :param forward_steps: int, the number of steps to move forward along the trajectory for potential direction
    """
    p = np.array(position)
    distances, forward_point = forward_point_on_trajectory(p, trajectory, forward_steps)# Gradient of the potential field directed towards the forward point
    if distances > 0.1:
        grad_x = -k_att * (p[0] - forward_point[0])
        grad_y = -k_att * (p[1] - forward_point[1]) 
        grad_z = -k_att * (p[2] - forward_point[2])   
        force = np.array([-grad_y, grad_x, grad_z, 0,0,0])
    else:
        force = np.array([0,0,0, 0,0,0])
    return forward_point, force


