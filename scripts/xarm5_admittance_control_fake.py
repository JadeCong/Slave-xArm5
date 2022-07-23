import sys, os, time
# sys.path.append(os.path.join(os.path.dirname(__file__), "../../conarobot_dependencies/xArm-Python-SDK"))

try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

import rospy
import tf
from geomeetry_msgs.msg import Wrench, WrenchStamped, Transform, TransformStamped, Pose, PoseStamped, Quaternion, Point

import numpy as np
import math


# Define the global variables for xarm5
xarm5_ft_data = Wrench()
xarm5_pose_data = Pose()


# Define the specific print function
def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)


# Fake ft sensor data callback
def xarm5_ft_callback(data):
    fake_ft_data = data.wrench


# xarm5 pose data callback
def xarm5_pose_callback(data):
    xarm5_pose_data = data.pose


# configure the admittance control parameters and update the xarm5 pose data
def xarm5_admittance_control_server():
    pass


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
# arm.set_position(*[450, 0, 200, 180, 0, 0], wait=True)  # center=[450, 0, 200, 180, 0, 0], radius=150

# Enable the servo motion mode
arm.set_mode(1)
arm.set_state(state=0)
time.sleep(0.05)

# Configure the ROS subscriber and publisher for the xarm5
rospy.init_node('xarm5_admittance_control_fake', anonymous=True)
xarm5_ft_sub = rospy.Subscriber('/fake_wrench', WrenchStamped, xarm5_ft_callback)
xarm5_pose_sub = rospy.Subscriber('/xarm5/pose', PoseStamped, xarm5_pose_callback)
xarm5_joint_pub = rospy.Publisher('/xarm5_joint_states', PoseStamped, queue_size=10)

# Runing in real time with fake force/torque sensor
# start = time.time()
idx = 0
while arm.connected and arm.state != 4:
    # calculate the target cartesian pose
    # delta_time = time.time() - start
    delta_time = idx / 100
    
    new_pose = arm.get_pose()
    
    mvpose = [x, y, z, 180, 0, 0]
    # mvpose = [x, y, z, rx, ry, rz]
    
    # set the target cartesian pose
    rtn = arm.set_servo_cartesian(mvpose, speed=100, mvacc=2000)
    pprint('Time:{}, pose:{}, set_servo_cartesian, rtn={}'.format(delta_time, mvpose, rtn))
    time.sleep(0.01)
    
    # check the return point
    idx += 1
    if idx >= 30000:  # loop for 30 seconds(frequency:100Hz(time.sleep(0.01)))
        pprint('Loop for 300 seconds, exit...')
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
