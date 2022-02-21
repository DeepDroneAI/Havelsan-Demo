from os import stat
import re
import airsim
import numpy as np
from geom_utils import QuadPose
import math

class Airsim_Updater:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.__prepare_drones()
        self.initial_poses=np.array([[0,0,0],[0,4,0],[0,8,0],[0,12,0]])
        self.ref_frame=[-30,30,0]
        self.index=0

    def __prepare_drones(self):
        for i in range(4):
            self.client.enableApiControl(True, f"Drone{i+1}")
        for i in range(4):
            self.client.armDisarm(True, f"Drone{i+1}")

    def set_drone_locs(self,pose):
        pose=self.__convert_locs(pose)
        for i in range(4):
            quad_pose=[pose[i][0],pose[i][1],-pose[i][2],0,0,0]
            self.client.simSetVehiclePose(QuadPose(quad_pose), True,f"Drone{i+1}")

    def update_agent_pose(self,pose):
        
        for i in range(2):
            if self.index==0:
                quad_pose=[pose[i][0]-self.initial_poses[i][0],pose[i][1]-self.initial_poses[i][1],-pose[i][2]-self.initial_poses[i][2],0,0,0]
            self.client.simSetVehiclePose(QuadPose(quad_pose), True,f"Drone{i+1}")

    def __convert_locs(self,pose):
        ref_pose=np.zeros((4,3))
        for i in range(4):
            for j in range(3):
                ref_pose[i][j]=pose[i][j]-self.initial_poses[i][j]
        return ref_pose

    def get_pose(self,index):
        state = self.client.getMultirotorState(f"Drone{index}")
        pose=[state.kinematics_estimated.position.x_val,state.kinematics_estimated.position.y_val,state.kinematics_estimated.position.z_val]

        return pose

    def euler_from_quaternion(self,quan):
        x, y, z, w=quan

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians




