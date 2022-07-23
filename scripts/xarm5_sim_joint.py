import os
import time
import sys
# sys.path.append(os.path.join(os.path.dirname(__file__), "../../conarobot_dependencies/xArm-Python-SDK"))

try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

import pybullet as pb
import pybullet_data as pbd
import numpy as np
import math


# Define the specific print function
def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)


# Configure the robot ip
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../config/xarm5.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit...')
            sys.exit(1)


# Connect the robot and enable motion
arm = XArmAPI(ip)
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)

# Go to target pose in position control mode
arm.set_mode(0)
arm.set_state(state=0)
arm.reset(wait=True)
# arm.set_position(*[600, 0, 200, 180, 0, 0], wait=True)  # center=[450, 0, 200, 180, 0, 0], radius=150

# Enable the servo motion mode
arm.set_mode(1)
arm.set_state(0)
time.sleep(0.05)


# Connect the bullet engine
bullet_client = pb.connect(pb.GUI)
# bullet_client = pb.connect(pb.DIRECT)  # no gui graphics
connect_rtn = pb.getConnectionInfo(bullet_client)
if connect_rtn["isConnected"] == 0:
    print('Failed to connect to the bullet engine...')
    sys.exit(1)
elif connect_rtn["isConnected"] == 1:
    print('Connected to the bullet engine...')

# Set the gravity
pb.setGravity(0, 0, -9.8)

# Load the robot urdf
xarm5 = pb.loadURDF(fileName="../model/xarm_description/urdf/xarm5_robot.urdf", useFixedBase=True, flags=pb.URDF_USE_SELF_COLLISION, physicsClientId=bullet_client)

# Configure the user debug parameters for xarm5 joint pose
jointIds = []
paramIds = []
for i in range(pb.getNumJoints(xarm5)):
    pb.changeDynamics(xarm5, i, linearDamping=0, angularDamping=0)
    info = pb.getJointInfo(xarm5, i)
    jointName =info[1]
    jointType =info[2]
    if (jointType == pb.JOINT_PRISMATIC or jointType == pb.JOINT_REVOLUTE):
        jointIds.append(i)
        paramIds.append(pb.addUserDebugParameter(jointName.decode("utf-8"), -3.14, 3.14, 0))
skip_cam_frames = 10

# Set the target joint pose form the user debug parameter server and send to the xarm5 robot for execution
while(pb.isConnected()):
    pb.stepSimulation()
    targetPositions = []
    for i in range(len(jointIds)):
        c = paramIds[i]
        targetPos = pb.readUserDebugParameter(c)
        pb.setJointMotorControl2(bodyIndex=xarm5, jointIndex=jointIds[i], controlMode=pb.POSITION_CONTROL, targetPosition=targetPos, force=500)
        targetPositions.append(targetPos)
    skip_cam_frames -= 1
    arm.set_servo_angle_j(angles=targetPositions, speed=math.radians(50), is_radian=True)
    if skip_cam_frames < 0:
        pb.getCameraImage(320, 200, renderer=pb.ER_BULLET_HARDWARE_OPENGL)
        skip_cam_frames = 20
    time.sleep(0.01)

arm.reset(wait=True)
arm.disconnect()
