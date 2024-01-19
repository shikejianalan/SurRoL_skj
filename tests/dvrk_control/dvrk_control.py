import numpy as np
import dvrk
import PyKDL
from dvrk import mtm

def move_to_target_forcebased(mtm, target_position):
    move_state = mtm.setpoint_cp()
    current_position = move_state.p
    diff = np.array(target_position) - np.array(current_position)
    distance = np.linalg.norm(np.array([diff.x(), diff.y(), diff.z()])) 
    print(distance)
    i = 0
    scale = 1
    while (distance > 1e-2) :
        force_scale = 10000.0 * distance *scale
        # print(force_scale)
        force = np.array([diff.x(), diff.y(), diff.z(), 0, 0, 0])* force_scale
        # print(force)
        mtm.body.servo_cf(force)

        current_position = mtm.setpoint_cp().p
        diff = np.array(target_position) - np.array(current_position)
        distance = np.linalg.norm(np.array([diff.x(), diff.y(), diff.z()]))
        i += 1
        # if i > 50000 :
        #     scale = 10
        # print(m.body.measured_cf())
        print(distance)   

    # m.move_cp(target_position)
    # force0 = np.array([0, 0, 0, 0, 0, 0])
    # mtm.body.servo_cf(force0)
    print('over')