import sys, os, time

import pybullet as pb
import pybullet_data as pbd
from xarm5_ik_bullet import XArm5IK_Bullet
# from xarm5_ik_ikfast import XArm5IK_IKFast
import numpy as np
import math


# Configure the bullet client
pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pbd.getDataPath())
time_step = 0.01  # frequency:100Hz
pb.setTimeStep(time_step)
pb.setGravity(0, 0, -9.8)

xarm5_base_pos = [0.0, 0.0, 0.0]
xarm5_base_ori = [0.0, 0.0, 0.0, 1.0]
xarm5_useNullSpace = False
xarm5_useDynamics = True
xarm5_ik = XArm5IK_Bullet(pb, xarm5_base_pos, xarm5_base_ori, xarm5_useNullSpace, xarm5_useDynamics, [0.3, 0.0, 0.2], [1, 0, 0, 0], time_step)

# Running sim demo
idx = 0
while pb.isConnected():
    xarm5_ik.step()
    pb.stepSimulation()
    time.sleep(time_step)  # loop rate: 100Hz
    
    idx += 1
    if idx >= 3000:
        print('Loop for 30 seconds, exit...')
        break

# Disconnect the bullet server
print("Discont with the bullet server...")
pb.disconnect()
print("Demo Done.")
