import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle
traj = np.load('/home/kj/skjsurrol/SurRoL_skj/tests/absolute_peg_transfer_position.npy')
print(traj.shape)
sim_distance = 0
for i in range(traj.shape[0]):
    if i != 0:
        sim_diff = traj[i] - traj[i-1]
        sim_distance += np.linalg.norm(sim_diff) 
        print('sim_diff:',sim_diff,'deta_sim:',np.linalg.norm(sim_diff)) 
        # print('deta_real:',np.linalg.norm(real_diff),'deta_sim:',np.linalg.norm(sim_diff))
        # print('real_distance:',real_distance,'sim_distance:',sim_distance)
print('sim_distance:',sim_distance)
exit()
file = open('/home/kj/skjsurrol/SurRoL_skj/tests/record.pkl','rb')
data = pickle.load(file)
file.close()
current_pos = []
target_pos = []
potential_force = []
mr_force = []
t = []
print(data)
for idx, i in enumerate(data):
    # if idx % 2 == 0:
        current_pos.append(i['current_pos'])
        target_pos.append(i['target_pos'])
        potential_force.append(i['potential_force'])
        mr_force.append(i['mr_force'])
        t.append(i['time'])
current_pos = np.array(current_pos)
target_pos = np.array(target_pos)
potential_force = np.array(potential_force)
mr_force = np.array(mr_force)
t = np.array(t)
print(t.shape)
print(mr_force[:,0].shape)
# lowest_point_idx = np.argmin(target_pos[:, 2])
# lowest_point = target_pos[lowest_point_idx]
# print(lowest_point)
# print(data[0]['mr_force'][0])


fig = plt.figure(figsize=(10, 8))
plt.title('mr_force')
plt.xlabel('time/s')
plt.ylabel('force/N')
plt.plot(t,mr_force[:,0],label = 'x')
plt.plot(t,mr_force[:,1],label = 'y')
plt.plot(t,mr_force[:,2],label = 'z')
plt.show()