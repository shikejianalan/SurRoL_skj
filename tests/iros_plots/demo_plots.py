import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle

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
    print(forward_index)
    return trajectory[forward_index]

def plot_3d_trajectory_forward_field(trajectory, k_att, grid_size=20, forward_steps=5):
    """
    Plot the 3D potential field with arrows pointing towards a forward point on the trajectory.    
    :param trajectory: numpy array of shape (n, 2), the trajectory points (x, y)
    :param k_att: float, scaling factor for the attractive potential
    :param grid_size: int, the size of the grid for visualization
    :param forward_steps: int, the number of steps to move forward along the trajectory for potential direction
    """
    x = np.linspace(-10, 10, grid_size)
    y = np.linspace(-10, 10, grid_size)
    z = np.linspace(-10, 10, grid_size)
    X, Y, Z = np.meshgrid(x, y, z)    
    U_grad_x = np.zeros(X.shape)
    U_grad_y = np.zeros(Y.shape)
    U_grad_z = np.zeros(Z.shape)   
    for i in range(grid_size):
        for j in range(grid_size):
            for k in range(grid_size):
                p = np.array([X[i, j, k], Y[i, j, k], Z[i, j, k]])
                forward_point = forward_point_on_trajectory(p, trajectory, forward_steps)            # Gradient of the potential field directed towards the forward point
                grad_x = -k_att * (p[0] - forward_point[0])
                grad_y = -k_att * (p[1] - forward_point[1]) 
                grad_z = -k_att * (p[2] - forward_point[2])              
                U_grad_x[i, j, k] = grad_x
                U_grad_y[i, j, k] = grad_y    
                U_grad_z[i, j, k] = grad_z    

    # fig = plt.figure(figsize=(10, 8))
    # ax = fig.add_subplot(111, projection='3d')
    # ax.quiver(X, Y, Z, U_grad_x, U_grad_y, U_grad_z, color='blue')
    # ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='red', linestyle='--', label='Trajectory')
    # ax.set_xlabel('X Axis')
    # ax.set_ylabel('Y Axis')
    # ax.set_zlabel('Z Axis')
    # ax.set_title('3D Potential Field Directed Towards Forward Point on Trajectory')
    # ax.legend()
    # # ax.grid(True)
    # plt.show()# Plot the 3D potential field with the curved trajectory, directing arrows towards a forward point

def generate_curved_trajectory(start_point, end_point, curvature, num_points=100):
    """
    Generate a curved trajectory between two points.    
    :param start_point: numpy array, the start point of the trajectory (x, y)
    :param end_point: numpy array, the end point of the trajectory (x, y)
    :param curvature: float, the curvature of the trajectory (positive for convex, negative for concave)
    :param num_points: int, the number of points in the trajectory
    :return: numpy array of shape (num_points, 2), the curved trajectory points (x, y)
    """
    t = np.linspace(0, 1, num_points)
    trajectory_x = (1 - t) * start_point[0] + t * end_point[0]
    trajectory_y = (1 - t) * start_point[1] + t * end_point[1] #+ curvature * np.sin(np.pi * t)
    trajectory_z = (1 - t) * start_point[2] + t * end_point[2] #+ curvature * np.cos(np.pi * t)
    return np.vstack([trajectory_x, trajectory_y, trajectory_z]).T# Define start and end points for the curved trajectory

def plot_figure(traj_path, data_path, task='needle_reach'):
    with open(data_path, 'rb') as f:
        data = pickle.load(f)

    current_pos = []
    all_pos = []
    target_pos = []
    potential_force = []
    scalar_potential_force = []
    mr_force = []
    t = []
    distance = []
    real_distance = 0
    all_distance = 0

    for idx, i in enumerate(data[:]):
        if idx % 1 == 0:
            current_pos.append(i['current_pos'][:3])
            all_pos.append(i['current_pos'])
            target_pos.append(i['target_pos'])
            potential_force.append(i['force'])
            scalar_potential_force.append(np.linalg.norm(i['force']))
            distance.append(i['distance'])
            # mr_force.append(i['mr_force'])
            # t.append(i['time'])
        else:
            if i['distance'] > 0.05:
                current_pos.append(i['current_pos'][:3])
                all_pos.append(i['current_pos'])
                target_pos.append(i['target_pos'])
                potential_force.append(i['force'])
                scalar_potential_force.append(np.linalg.norm(i['force']))
                distance.append(i['distance'])
    current_pos = np.array(current_pos)
    target_pos = np.array(target_pos)
    potential_force = np.array(potential_force)
    scalar_potential_force = np.array(scalar_potential_force)

    traj = np.load(traj_path)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(current_pos[:, 0], current_pos[:, 1], current_pos[:, 2], potential_force[:, 1], -potential_force[:, 0], potential_force[:, 2], color='red', length = 0.03)
    ax.scatter(current_pos[:, 0], current_pos[:, 1], current_pos[:, 2], color='blue', linestyle='--',alpha=0.5)
    ax.plot(current_pos[:, 0], current_pos[:, 1], current_pos[:, 2], color='blue', linestyle='-',  label='Actual PSM Position',linewidth=1)

    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color='green', linestyle='-', label='RL-predicted Trajectory', linewidth=3)
    ax.scatter(traj[:, 0], traj[:, 1], traj[:, 2], color='green', linestyle='-', linewidth=3)
    ax.scatter(traj[0, 0], traj[0, 1], traj[0, 2], color='red',s=150)
    ax.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2], color='lightgreen',s=150)
    ax.view_init(elev=38, azim=20)
    if task != 'needle_reach':
        minimum_point_id = np.argmin(traj[...,2])
        minimum_point = traj[minimum_point_id]
        ax.scatter(minimum_point[0], minimum_point[1], minimum_point[2], color='orange',s=150)
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.legend()
    ax.grid(True)
    plt.show()

    fig2, axs = plt.subplots(2, figsize=(10, 3), sharex=True)
    # ax1 = fig.add_subplot(211)
    ax1=axs[0]
    ax1.plot( np.arange(len(distance)),np.array(distance), color='blue', linestyle='-')
    # ax1.set_ylabel('Deviation from \n the Trajectory', fontsize=8)
    ax1.axhline(0.05, color='red', linestyle='--', label='Distance Threshold')
    # ax2 = fig.add_subplot(212)
    ax2=axs[1]
    ax2.plot( np.arange(len(distance)),scalar_potential_force, color='blue', linestyle='-', label='distance')
    # ax2.set_ylabel('Force Magnitude',fontsize=8)
    # ax2.set_xlabel('Time Steps')
    # ax.set_title('3D Potential Field Directed Towards Forward Point on Trajectory')
    ax1.legend()

    plt.show()# Plot the 3D potential field with the curved trajectory, directing arrows towards a forward point
    

plot_figure('./absolute_needle_reach_position.npy','./demo_record_needle_reach_demo_plot2.pkl', task='needle_reach')
plot_figure('./absolute_gauze_retrieve_position.npy','./demo_record_gauze_retrieve_demo_plot2.pkl', task='gauze_retrieve')