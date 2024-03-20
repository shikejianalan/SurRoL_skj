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

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(X, Y, Z, U_grad_x, U_grad_y, U_grad_z, color='blue')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='red', linestyle='--', label='Trajectory')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('3D Potential Field Directed Towards Forward Point on Trajectory')
    ax.legend()
    # ax.grid(True)
    plt.show()# Plot the 3D potential field with the curved trajectory, directing arrows towards a forward point

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

# start_point = np.array([-5, -5, -5])
# end_point = np.array([5, 5, 5])
# curvature = 5  # Adjust curvature to change the shape of the curve# Generate the curved trajectory
# curved_trajectory = generate_curved_trajectory(start_point, end_point, curvature)
# print(curved_trajectory.shape)

# plot_3d_trajectory_forward_field(curved_trajectory, 0.1, grid_size=10, forward_steps=5)
# /home/kj/skjsurrol/SurRoL_skj/tests/position_and_force_new.pkl
# with open('/home/kj/skjsurrol/SurRoL_skj/tests/record/needle_test/demo_record_needle_reach_test_8.pkl', 'rb') as f:
# with open('/home/kj/skjsurrol/SurRoL_skj/tests/record/gauze_test/demo_record_gauze_retrive_test_4.pkl', 'rb') as f:
with open('/home/kj/skjsurrol/SurRoL_skj/tests/record/wenhei/needle_reach_wenhei_test3_1.pkl', 'rb') as f:
# with open('/home/kj/skjsurrol/SurRoL_skj/tests/record/demo_record_gauze_retrive_test_11.pkl', 'rb') as f:
    data = pickle.load(f)
# data = np.load('/home/kj/skjsurrol/SurRoL_skj/tests/absolute_needle_pick_position.npy')
f.close
# minimum_point = np.argmin(data[...,2])
# print(data[-1])
# print(data[minimum_point])
current_pos = []
all_pos = []
potential_force = []
mr_force = []
t = []
real_distance = 0
all_distance = 0

for idx, i in enumerate(data):
    # if idx % 2 == 0:
        current_pos.append(i['current_pos'][:3])
        all_pos.append(i['current_pos'])
        # target_pos.append(i['target_pos'])
        # potential_force.append(i['force'])
        # mr_force.append(i['mr_force'])
        # t.append(i['time'])

current_pos = np.array(current_pos)
# target_pos = np.array(target_pos)
# potential_force = np.array(potential_force)
# mr_force = np.array(mr_force)
# t = np.array(t)
for i in range(current_pos.shape[0]):
    if i != 0:
        real_diff = current_pos[i] - current_pos[i-1]
        real_distance += np.linalg.norm(real_diff)
        all_diff = all_pos[i] - all_pos[i-1]
        all_distance += np.linalg.norm(all_diff)

        # sim_diff = target_pos[i] - target_pos[i-1]
        # sim_distance += np.linalg.norm(sim_diff) 
print('real_distance:',real_distance)
print('sim_distance:',all_distance)


# for idx, i in enumerate(data):
#     # if idx % 2 == 0:
#         current_pos.append(i['current_pos'])
#         target_pos.append(i['target_pos'])
        # potential_force.append(i['potential_force'])
        # mr_force.append(i['mr_force'])
        # t.append(i['time'])
# current_pos = np.array(current_pos)
# target_pos = np.array(target_pos)
# print(target_pos.shape)
# lowest_point_idx = np.argmin(target_pos[:, 2])
# lowest_point = target_pos[lowest_point_idx]
# print(lowest_point)
# force = np.array(force)


# traj = np.load('/home/kj/skjsurrol/SurRoL_skj/tests/absolute_peg_transfer_position.npy')



fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
# ax.quiver(current_pos[:, 0], current_pos[:, 1], current_pos[:, 2], potential_force[:, 1], -potential_force[:, 0], potential_force[:, 2], color='red', length = 0.05)
# ax.scatter(lowest_point[0], lowest_point[1], lowest_point[2], color='red', label='lowest point')
# ax.scatter(lowest_point[0], lowest_point[1], 3.65, color='red', label='lowest point')
# ax.scatter(data[minimum_point][0], data[minimum_point][1], data[minimum_point][2], color='red', linestyle='--', label='current_pos')
ax.scatter(current_pos[:, 0], current_pos[:, 1], current_pos[:, 2], color='blue', linestyle='--', label='current_pos')
# ax.plot(target_pos[:, 0], target_pos[:, 1], target_pos[:, 2], color='green', linestyle='-', label='start')

# ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color='darkgreen', linestyle='-', linewidth=5, label='target_pos')
# ax.plot(data[..., 0], data[..., 1], data[..., 2])
# ax.plot(potential_force[:, 0], potential_force[:, 1], potential_force[:, 2], color='blue', linestyle='--', label='force')
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title('3D Potential Field Directed Towards Forward Point on Trajectory')
ax.legend()
# ax.grid(True)
plt.show()# Plot the 3D potential field with the curved trajectory, directing arrows towards a forward point