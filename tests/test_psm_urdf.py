import pybullet as p
import time
urdf = '/home/kj/skjsurrol/SurRoL_skj/surrol/assets/psm/psm.urdf'

physicsClient = p.connect(p.GUI)
# p.setGravity(0,0,-9.81)
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
planeId = p.loadURDF(urdf, cubeStartPos, cubeStartOrientation)

while True: 
    p.stepSimulation()
    time.sleep(1./240.)