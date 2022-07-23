import sys, os, time
import datetime
import random
import traceback
import threading

try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

import numpy as np
import math

from socket import socket, AF_INET, SOCK_DGRAM
import json


# Define the udp server
net_addr = "127.0.0.1"
net_port = 8080
net_info = (net_addr, net_port)
buf_size = 1024

sock_server = socket(AF_INET, SOCK_DGRAM)
sock_server.bind(net_info)


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
arm.set_position(*[300, 0, 200, 180, 0, 0], wait=True)  # head down 
# arm.set_position(*[500, 0, 500, 180, -90, 0], wait=True)  # head forward

# Enable the servo motion mode
arm.set_mode(1)
arm.set_state(state=0)
time.sleep(0.05)

# Runing in circle with end effector
control_origin_pose = arm.get_position(is_radian=True)[1]
while arm.connected and arm.state != 4:
    # receive the pose cmd from the master robot(Touch)
    msg, addr = sock_server.recvfrom(buf_size)
    msg_dict = json.loads(msg.decode('utf-8'))
    print('Got message({}) from'.format(msg_dict), addr)
    
    # parse the pose
    cmd_index, cmd_flag, cmd_pose = msg_dict['index'], msg_dict['flag'] , msg_dict['pose']
    if cmd_index != 0:  # check whether the robot is under control
        pass
    elif cmd_index == 0:
        control_origin_pose = arm.get_position(is_radian=True)[1]
    
    # remap the position between master(Touch:[x,y,z]) and slave(xArm5:[y,z,x])
    cmd_pos = [-1 * cmd_pose['pos'][2]] + cmd_pose['pos'][0:2]
    # remap the orientation between master(Touch:[yaw,-pitch,-roll]) and slave(xArm5:[roll,pitch,yaw])
    # cmd_ori = [-1 * cmd_pose['ori'][2]] +  [-1 * cmd_pose['ori'][1]] + [cmd_pose['ori'][0]]  # add the orientation(roll/pitch/yaw angle)
    cmd_ori = [0] +  [-1 * cmd_pose['ori'][1]] + [cmd_pose['ori'][0]]  # pass the roll angle
    # cmd_ori = [0, 0, 0]  # pass the orientation(roll/pitch/yaw angle)
    
    if cmd_flag == 'abs':  # absolute pose mode
        mvpose = cmd_pos + cmd_ori  # control the position and orientation
    elif cmd_flag == 'rel':  # relative pose mode
        mvpose = list(map(lambda x: x[0]+x[1], zip(control_origin_pose[0:3], cmd_pos))) + list(map(lambda y: y[0]+y[1], zip(control_origin_pose[3:6], cmd_ori)))  # control the position and orientation
    
    # move the robot
    rtn = arm.set_servo_cartesian(mvpose, speed=100, mvacc=2000, is_radian=True)
    pprint('Pose:{}, set_servo_cartesian, rtn={}'.format(mvpose, rtn))
    time.sleep(0.005)
    
    # get the contact force form the slave robot(xArm5)
    # joint_torques = arm.get_joints_torque()
    # ee_ft_data = arm.get_ft_sensor_data()
    
    # send the contact force to the master robot(Touch)
    response = "Get and set the pose successfully!"
    sock_server.sendto(response.encode('utf-8'), addr)

# Go to home
arm.set_mode(0)
arm.set_state(state=0)
arm.reset(wait=True)
pprint("Going home...")

# Disconnect with the robot
pprint("Discont with the robot...")
arm.disconnect()
pprint("Demo Done.")
