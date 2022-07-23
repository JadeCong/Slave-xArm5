import sys, os, time
# sys.path.append(os.path.join(os.path.dirname(__file__), "../../conarobot_dependencies/xArm-Python-SDK"))

try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

import pybullet as pb
import pybullet_data as pbd
from xarm5_ik_bullet import XArm5IK_Bullet
# from xarm5_ik_ikfast import XArm5IK_IKFast
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
xarm5_ik = XArm5IK_Bullet(pb, xarm5_base_pos, xarm5_base_ori, xarm5_useNullSpace, xarm5_useDynamics, [0.3, 0.0, 0.2], [1, 0, 0, 0], time_step)  # # quaternion:[1, 0, 0, 0] = euler angle:[180, 0, 0]

# Connect the robot and enable motion
arm = XArmAPI(ip)
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)

# Go to target pose in position control mode
arm.set_mode(0)
arm.set_state(state=0)
arm.reset(wait=True)
arm.set_position(*[300, 0, 200, 180, 0, 0], wait=True)  # center=[450, 0, 200, 180, 0, 0], radius=150

# Enable the servo motion mode
arm.set_mode(1)
arm.set_state(state=0)
time.sleep(0.05)

# Runing in circle with end effector
# idx = 0
while pb.isConnected() and arm.connected and arm.state != 4:
    xarm5_ik.step()
    rtn = arm.set_servo_angle_j(angles=xarm5_ik.xarm5_joint_pose, speed=math.radians(50), is_radian=True)
    pprint('Time:{}, pose:{}, set_servo_angle_j, rtn={}'.format(xarm5_ik.time, xarm5_ik.xarm5_ee_pose, rtn))
    pb.stepSimulation()
    time.sleep(time_step)  # loop rate: 100Hz
    
    # idx += 1
    # if idx >= 3000:  # loop for 30 seconds(frequency:100Hz(time.sleep(0.01)))
    if xarm5_ik.time >= 30:
        pprint('Loop for 30 seconds, exit...')
        break

# Go to home
arm.set_mode(0)
arm.set_state(state=0)
arm.reset(wait=True)
pprint("Going home...")

# Disconnect with the robot
pprint("Discont with the robot...")
arm.disconnect()
pprint("Demo Done.")
