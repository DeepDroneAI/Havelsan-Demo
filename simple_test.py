import airsim
import numpy as np
from airsim_update import Airsim_Updater



updater=Airsim_Updater()

pose=[[0,0,1.2],[0,0,1],[0,0,1.1],[0,0,1.3]]

updater.set_drone_locs(pose)
updater.get_states(2)