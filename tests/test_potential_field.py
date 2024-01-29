import numpy as np
import matplotlib.pyplot as plt
import time
def forward_point_on_trajectory(p, trajectory, forward_steps=5):
    """
    Find a forward point on the trajectory from the nearest point to p.    :param p: numpy array, the position vector of the current point (x, y)
    :param trajectory: numpy array of shape (n, 2), the trajectory points (x, y)
    :param forward_steps: int, the number of steps to move forward along the trajectory
    :return: numpy array, the position vector of the forward point (x, y)
    """
    # Find the nearest point index on the trajectory
    distances = np.linalg.norm(trajectory - p, axis=1)
    nearest_index = np.argmin(distances)    # Calculate the forward index, ensuring it doesn't exceed the trajectory length
    forward_index = min(nearest_index + forward_steps, len(trajectory) - 1)    
    return trajectory[forward_index]

def plot_2d_trajectory_forward_field(trajectory, k_att, grid_size=20, forward_steps=5):
    """
    Plot the 2D potential field with arrows pointing towards a forward point on the trajectory.    :param trajectory: numpy array of shape (n, 2), the trajectory points (x, y)
    :param k_att: float, scaling factor for the attractive potential
    :param grid_size: int, the size of the grid for visualization
    :param forward_steps: int, the number of steps to move forward along the trajectory for potential direction
    """
    x = np.linspace(-10, 10, grid_size)
    y = np.linspace(-10, 10, grid_size)
    X, Y = np.meshgrid(x, y)    
    U_grad_x = np.zeros(X.shape)
    U_grad_y = np.zeros(Y.shape)    
    for i in range(grid_size):
        for j in range(grid_size):
            p = np.array([X[i, j], Y[i, j]])
            forward_point = forward_point_on_trajectory(p, trajectory, forward_steps)            # Gradient of the potential field directed towards the forward point
            grad_x = -k_att * (p[0] - forward_point[0])
            grad_y = -k_att * (p[1] - forward_point[1])           
            U_grad_x[i, j] = grad_x
            U_grad_y[i, j] = grad_y    

    # plt.figure(figsize=(8, 6))
    # plt.quiver(X, Y, U_grad_x, U_grad_y, color='blue')
    # plt.plot(trajectory[:, 0], trajectory[:, 1], color='red', linestyle='--', label='Trajectory')
    # plt.xlabel('X Axis')
    # plt.ylabel('Y Axis')
    # plt.title('2D Potential Field Directed Towards Forward Point on Trajectory')
    # plt.legend()
    # plt.grid(True)
    # plt.show()# Plot the 2D potential field with the curved trajectory, directing arrows towards a forward point

def generate_curved_trajectory(start_point, end_point, curvature, num_points=100):
    """
    Generate a curved trajectory between two points.    :param start_point: numpy array, the start point of the trajectory (x, y)
    :param end_point: numpy array, the end point of the trajectory (x, y)
    :param curvature: float, the curvature of the trajectory (positive for convex, negative for concave)
    :param num_points: int, the number of points in the trajectory
    :return: numpy array of shape (num_points, 2), the curved trajectory points (x, y)
    """
    t = np.linspace(0, 1, num_points)
    trajectory_x = (1 - t) * start_point[0] + t * end_point[0]
    trajectory_y = (1 - t) * start_point[1] + t * end_point[1] + curvature * np.sin(np.pi * t)
    return np.vstack([trajectory_x, trajectory_y]).T# Define start and end points for the curved trajectory

start_point = np.array([-5, -5])
end_point = np.array([5, 5])
curvature = 5  # Adjust curvature to change the shape of the curve# Generate the curved trajectory
curved_trajectory = generate_curved_trajectory(start_point, end_point, curvature)

start_time = time.time()
plot_2d_trajectory_forward_field(curved_trajectory, 10, grid_size=20, forward_steps=10)
end_time = time.time()

used = end_time - start_time
print(used)