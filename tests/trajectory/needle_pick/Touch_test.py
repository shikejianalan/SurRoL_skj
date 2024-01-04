import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')

MTM_traj = np.load("PickandPlace_pos_MTM2.npy")
waypoints =np.array(MTM_traj)
xs = waypoints[..., 2]
ys = waypoints[..., 0]
zs = waypoints[..., 1]

# xs = waypoints[..., 0]
# ys = waypoints[..., 1]
# zs = waypoints[..., 2]

ax.plot3D(xs, ys, zs, 'gray')
plt.show()

# print(MTM_traj)

