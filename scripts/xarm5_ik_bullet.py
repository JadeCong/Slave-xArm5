import time
import numpy as np
import math


class XArm5IK_Bullet(object):
    def __init__(self, bullet_client, base_pos=[0,0,0], base_ori=[0, 0, 0, 1], useNullSpace=False, useDynamics=True, ee_pos=[0.3, 0.0, 0.2], ee_ori=[1, 0, 0, 0], time_step=0.01):
        self.bullet_client = bullet_client
        self.base_pos = base_pos
        self.base_ori = base_ori
        self.useNullSpace = useNullSpace
        self.useDynamics = useDynamics
        self.ee_pos = ee_pos
        self.ee_ori = ee_ori
        self.time_step = time_step
        
        self.flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.xarm5 = self.bullet_client.loadURDF("model/xarm_description/urdf/xarm5_robot.urdf", self.base_pos, self.base_ori, useFixedBase=True, flags=self.flags)
        self.xarm5_dof_num = 5  # xarm5 dof:5
        self.xarm5_joint_num = self.bullet_client.getNumJoints(self.xarm5)
        self.xarm5_init_joint_pose = [0.0] * self.xarm5_dof_num
        self.xarm5_joint_pose = [0.0] * self.xarm5_dof_num
        self.null_ll = [-4] * self.xarm5_dof_num
        self.null_ul = [4] * self.xarm5_dof_num
        self.null_jr = [3.14] * self.xarm5_dof_num
        
        for i in range(self.xarm5_dof_num):
            self.bullet_client.changeDynamics(self.xarm5, i+1, linearDamping=0, angularDamping=0)
            info = self.bullet_client.getJointInfo(self.xarm5, i+1)
            jointName = info[1]
            jointType = info[2]
            if (jointType == self.bullet_client.JOINT_PRISMATIC or jointType == self.bullet_client.JOINT_REVOLUTE):
                self.bullet_client.resetJointState(self.xarm5, i+1, self.xarm5_init_joint_pose[i])
        
        self.start_time = 0.0
        self.time = self.start_time
        self.xarm5_ee_pos = self.ee_pos
        self.xarm5_ee_ori = self.ee_ori  # quaternion:[1, 0, 0, 0] = euler angle:[180, 0, 0]
        self.xarm5_ee_pose = self.xarm5_ee_pos + self.xarm5_ee_ori
        self.xarm5_ee_pos_prev = self.xarm5_ee_pos
        self.xarm5_ee_ori_prev = self.xarm5_ee_ori
        self.xarm5_ee_pose_prev = self.xarm5_ee_pose
    
    def step(self):
        # update the xarm5 ee pose for motion
        self.xarm5_ee_pos = (np.array([self.ee_pos[0]+0.15, self.ee_pos[1], self.ee_pos[2]]) + np.array([0.15*np.cos(self.time+math.pi), 0.15*np.sin(self.time+math.pi), 0.0])).tolist()  # center=[0.45, 0, 0.2, 180, 0, 0], radius=0.15
        self.xarm5_ee_pose = self.xarm5_ee_pos + self.xarm5_ee_ori
        self.time += self.time_step
        
        # calculate the xarm5 joint pose
        if self.useNullSpace:
            joint_pose = self.bullet_client.calculateInverseKinematics(self.xarm5, 5, self.xarm5_ee_pos, self.xarm5_ee_ori, lowerLimits=self.null_ll, upperLimits=self.null_ul, 
                         jointRanges=self.null_jr, restPoses=np.array(self.xarm5_init_joint_pose).tolist(), residualThreshold=1e-5, maxNumIterations=50)
            self.xarm5_joint_pose = joint_pose
        else:
            self.xarm5_joint_pose = self.bullet_client.calculateInverseKinematics(self.xarm5, 5, self.xarm5_ee_pos, self.xarm5_ee_ori, maxNumIterations=50)
        
        if self.useDynamics:
            for i in range(self.xarm5_dof_num):
                self.bullet_client.setJointMotorControl2(self.xarm5, i+1, self.bullet_client.POSITION_CONTROL, self.xarm5_joint_pose[i], force=5*240.)
        else:
            for i in range(self.xarm5_dof_num):
                self.bullet_client.resetJointState(self.xarm5, i+1, self.xarm5_init_joint_pose[i])
        
        # plot the xarm5 ee trajectory
        self.bullet_client.addUserDebugLine(self.xarm5_ee_pos_prev, self.xarm5_ee_pos, lineColorRGB=[1,0,0], lineWidth=2, lifeTime=100)
        # link_state = self.bullet_client.getLinkState(self.xarm5, 5, computeForwardKinematics=True)
        # link_urdf_pos = link_state[4]
        # self.bullet_client.addUserDebugLine(self.xarm5_ee_pos, link_urdf_pos, lineColorRGB=[1,0,0], lineWidth=2, lifeTime=100)
        
        # update the previous xarm5 ee pose
        self.xarm5_ee_pos_prev = self.xarm5_ee_pos
        self.xarm5_ee_ori_prev = self.xarm5_ee_ori
        self.xarm5_ee_pose_prev = self.xarm5_ee_pose
    
    def reset(self):
        pass
