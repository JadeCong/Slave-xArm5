import sys
# sys.path.append(os.path.join(os.path.dirname(__file__), "../../conarobot_dependencies/xArm-Python-SDK"))
import math
import time
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


def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

arm = XArmAPI('192.168.1.231')
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

variables = {}
params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}


# Register error/warn changed callback
def error_warn_change_callback(data):
    if data and data['error_code'] != 0:
        params['quit'] = True
        pprint('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)


# Register state changed callback
def state_changed_callback(data):
    if data and data['state'] == 4:
        if arm.version_number[0] >= 1 and arm.version_number[1] >= 1 and arm.version_number[2] > 0:
            params['quit'] = True
            pprint('state=4, quit')
            arm.release_state_changed_callback(state_changed_callback)
arm.register_state_changed_callback(state_changed_callback)


# Register counter value changed callback
if hasattr(arm, 'register_count_changed_callback'):
    def count_changed_callback(data):
        if not params['quit']:
            pprint('counter val: {}'.format(data['count']))
    arm.register_count_changed_callback(count_changed_callback)


# Register connect changed callback
def connect_changed_callback(data):
    if data and not data['connected']:
        params['quit'] = True
        pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
        arm.release_connect_changed_callback(error_warn_change_callback)
arm.register_connect_changed_callback(connect_changed_callback)


# Joint Motion
for i in range(int(3)):
    if params['quit']:
        break
    if not params['quit']:
        params['angle_speed'] = 40
    if not params['quit']:
        params['angle_acc'] = 800
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_servo_angle(angle=[0.0, 14.0, -25.0, 10.0, -0.6], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_servo_angle(angle=[-14.0, 42.8, -46.6, 13.0, -0.6], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_servo_angle(angle=[21.9, 50.0, -49.0, 30.0, -0.6], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        if code != 0:
            params['quit'] = True
            pprint('set_servo_angle, code={}'.format(code))


# Go Home
if params['quit']:
    pass
if not params['quit']:
    params['angle_speed'] = 40
if not params['quit']:
    params['angle_acc'] = 800
if arm.error_code == 0 and not params['quit']:
    code = arm.set_servo_angle(angle=[0.0, 0.0, 0.0, 0.0, 0.0], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
    if code != 0:
        params['quit'] = True
        pprint('set_servo_angle, code={}'.format(code))


# release all event
if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)
