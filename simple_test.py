import airsim
import numpy as np
from airsim_update import Airsim_Updater
from low_level_updates import Low_Level_Updates


low_control=Low_Level_Updates()

print(low_control.state_to_rl())

action=[4,4]
input("Start episode")

for i in range(100):
    statu=low_control.step(actions=action)
    if statu:
        break