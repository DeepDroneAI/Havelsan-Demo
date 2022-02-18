import airsim
import numpy as np
from airsim_update import Airsim_Updater



updater=Airsim_Updater()

pose=[[1,1,1],[1,1,1],[1,1,1],[1,1,1]]

updater.set_drone_locs(pose)