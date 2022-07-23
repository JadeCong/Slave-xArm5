import sys, os, time
import rospy
import tf
from geomeetry_msgs.msg import Wrench, WrenchStamped, Transform, TransformStamped, Pose, PoseStamped, Quaternion, Point
from sensor_msgs.msg import JointState


# Define the global variables for xarm5
fake_ft_data = Wrench()
xarm5_pose_data = Pose()


# Define the xarm5_ft_callback function
def xarm5_ft_callback(data):
    fake_ft_data = data.wrench

# Define the xarm5_pose_callback function
def xarm5_pose_callback(data):
    xarm5_pose_data = data.pose


# Initialize the ros node
rospy.init_node('xarm5_admittance_control_sim_server', anonymous=True)

# Configrue the subscriber for getting the fake ft sensor data and xarm5 pose data
rospy.Subscriber("/fake_wrench", WrenchStamped, xarm5_ft_callback)
rospy.Subscriber("/xarm5/pose", PoseStamped, xarm5_pose_callback)

# Configure the publisher for publishing the robot joint states
rospy.Publisher("/joint_states", JointState, queue_size=10)

# Configure the impedance control parameters
