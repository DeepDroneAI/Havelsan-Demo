import airsim
import numpy as np
from geom_utils import QuadPose


class Airsim_Updater:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("Connected to Airsim!!")
        self.__prepare_drones()

    def __prepare_drones(self):
        for i in range(4):
            self.client.enableApiControl(True, f"Drone{i+1}")
        for i in range(4):
            self.client.armDisarm(True, f"Drone{i+1}")

    def set_drone_locs(self,pose):
        for i in range(4):
            quad_pose=[pose[i][0],pose[i][1],-pose[i][2],0,0,0]
            self.client.simSetVehiclePose(QuadPose(quad_pose), True,f"Drone{i+1}")







