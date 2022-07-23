import sys, os, time
# sys.path.append(os.path.join(os.path.dirname(__file__), "../../conarobot_dependencies/xArm-Python-SDK"))

try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

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
arm.set_position(*[300, 0, 200, 180, 0, 0], wait=True)  # center=[450, 0, 200, 180, 0, 0], radius=150

# Enable the servo motion mode
arm.set_mode(1)
arm.set_state(state=0)
time.sleep(0.05)

# Runing in circle with end effector
# start = time.time()
idx = 0
while arm.connected and arm.state != 4:
    # calculate the target cartesian pose
    # delta_time = time.time() - start
    delta_time = idx / 100
    x = 450 + 150 * np.cos(delta_time+math.pi)
    y = 0 + 150 * np.sin(delta_time+math.pi)
    mvpose = [x, y, 200, 180, 0, 0]
    
    # get the target joint pose using inverse kinematics
    joint_rtn = arm.get_inverse_kinematics(mvpose, input_is_radian=False, return_is_radian=True)
    if joint_rtn[0] != 0:
        raise Exception('Failed to get inverse kinematics of current cartesian pose.')
        continue
    
    # set the target joint pose
    rtn = arm.set_servo_angle_j(joint_rtn[1], speed=math.radians(50), is_radian=True)
    pprint('Time:{}, pose:{}, set_servo_angle_j, rtn={}'.format(delta_time, mvpose, rtn))
    time.sleep(0.01)
    
    # check the return point
    idx += 1
    if idx >= 3000:  # loop for 30 seconds(frequency:100Hz(time.sleep(0.01)))
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
