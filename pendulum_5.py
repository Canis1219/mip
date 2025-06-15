import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

guiFlag = True

dt = 1/240
th0 = 0.5  
g = 10     
L = 0.8    
L1 = L
L2 = L
m = 1      
f0 = 10    

xd = 0.5   
zd = 1    

physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-g)

planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("simple.urdf.xml", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=th0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

pos0 = p.getLinkState(boxId, 4)[0]     
X0 = np.array([[pos0[0]],[pos0[2]]])

maxTime = 5 # seconds
logTime = np.arange(0, maxTime, dt)
sz = len(logTime)
logXsim = np.zeros(sz)
logZsim = np.zeros(sz)
idx = 0
T = 2
for t in logTime:
    Xd = np.array([[xd],[zd]])

    s = 1
    if t < T:
        s = (3/T**2) * t**2 - 2/(T**3) * t**3
    Xd_curr = X0 + s * (Xd - X0)
    new_pos = [Xd_curr[0], 0, Xd_curr[1]]

    result = p.calculateInverseKinematics(boxId, 4, new_pos)

    if len(result) >= 1:
        p.setJointMotorControl2(boxId, 1, p.POSITION_CONTROL, targetPosition=result[0]) 
    if len(result) >= 2:
        p.setJointMotorControl2(boxId, 3, p.POSITION_CONTROL, targetPosition=result[1]) 

    pos = p.getLinkState(boxId, 4)[0]
    logXsim[idx] = pos[0]
    logZsim[idx] = pos[2]

    p.stepSimulation()
    idx += 1
    if guiFlag:
        time.sleep(dt)

p.disconnect()

plt.subplot(2,1,1)
plt.plot(logTime, logXsim)
plt.ylabel("X position")
plt.subplot(2,1,2)
plt.plot(logTime, logZsim)
plt.ylabel("Z position")
plt.xlabel("Time [s]")
plt.show()