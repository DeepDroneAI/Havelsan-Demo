import airsim
import numpy as np
from airsim_update import Airsim_Updater
from low_level_updates import Low_Level_Updates


low_control=Low_Level_Updates()

pose=[[0,0,5],[0,4,5]]

low_control.set_target_poses(pose)

action=[4,4]
input("Start episode")

for i in range(100):
    low_control.step(actions=action)