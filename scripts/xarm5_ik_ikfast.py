import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), "./ikfastpy_xarm5"))

import ikfast_xarm5

import numpy as np
import math
from scipy.spatial.transform import Rotation as R


class XArm5IK_IKFast(object, init_joint_pose):
    def __init__(self, time_step):
        self.xarm5_kinematics = ikfast_xarm5.PyKinematics()
        self.xarm5_joint_num = self.xarm5_kinematics.getDOF()
        
        self.xarm5_ee_pose = None
        self.xarm5_joint_configs = None
        self.num_solution = None
        self.xarm5_joint_solutions = None
        self.xarm5_joint_pose = init_joint_pose
        self.xarm5_joint_pose_prev = self.xarm5_joint_pose
    
    def step(self, ee_pose):
        # format the ee_pose into matrix(transform:3x4)
        ee_ori = R.from_quat(ee_pose[3:]).as_matrix()  # quat:[x,y,z,w]
        self.xarm5_ee_pose = np.concatenate((ee_ori, np.array(ee_pose[:3]).reshape(3,1)), axis=1)  # ee_pose(matrix:3x4)
        
        # calculate the xarm5 joint configs
        self.xarm5_joint_configs = self.xarm5_kinematics.inverse(self.xarm5_ee_pose.reshape(-1).tolist())
        
        # choose the xarm5 joint pose which is closest to the previous joint pose
        self.num_solution = int(len(self.xarm5_joint_configs)/self.xarm5_joint_num)
        if self.num_solution < 1:
            print("No ik solution found, process next iteration...")
            pass
        
        self.xarm5_joint_solutions = np.asarray(self.xarm5_joint_configs).reshape(self.num_solution, self.xarm5_joint_num)
        joint_errors = [np.sum(np.abs(joint_pose - np.asarray(self.xarm5_joint_pose_prev))) for joint_pose in self.xarm5_joint_solutions]
        joint_pose_optimal = self.xarm5_joint_solutions[np.argmin(joint_errors)]
        self.xarm5_joint_pose = joint_pose_optimal.tolist()
        print("Found ik solution: {}".format(self.xarm5_joint_pose))
        
        # update the xarm5 joint pose_prev
        self.xarm5_joint_pose_prev = self.xarm5_joint_pose
    
    def reset(self):
        pass
