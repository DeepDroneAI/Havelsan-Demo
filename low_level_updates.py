import numps as np
from trajectory import Trajectory
from quadrotor import *
from airsim_update import Airsim_Updater

class Low_Level_Updates:
    def ___init__(self):
        self.updater=Airsim_Updater()
        self.quad1=Quadrotor(state0=self.__pose_to_state(self.updater.initial_poses[0]))
        self.quad2=Quadrotor(state0=self.__pose_to_state(self.updater.initial_poses[1]))
        self.trajSelect=np.array([3,2,1])
        self.controller="Backstepping_3"
        

    def __pose_to_state(self,pose):
        return [pose[0],pose[1],pose[2],0,0,0,0,0,0,0,0,0]

    def get_poses(self):
        poses=np.array((4,3))
        for i in range(4):
            poses[i]=self.updater.get_pose(index=i)
        return poses

    def set_target_poses(self,poses):
        all_poses=[self.updater.initial_poses[0],self.updater.initial_poses[1],poses[0],poses[1]]
        self.updater.set_drone_locs(all_poses)

    def action_to_target(self,actions,target1,target2):
        if actions[0]==0:
            target1[0]+=1
        elif actions[0]==1:
            target1[0]-=1
        elif actions[0]==2:
            target1[1]+=1
        elif actions[0]==3:
            target1[1]-=1
        elif actions[0]==4:
            target1[2]+=1
        elif actions[0]==5:
            target1[2]-=1


        if actions[1]==0:
            target2[0]+=1
        elif actions[1]==1:
            target2[0]-=1
        elif actions[1]==2:
            target2[1]+=1
        elif actions[1]==3:
            target2[1]-=1
        elif actions[1]==4:
            target2[2]+=1
        elif actions[1]==5:
            target2[2]-=1

        return target1,target2

    def step(self,actions):
        target=self.get_poses()
        target1=target[0]
        target2=target[1]
        pose1=target1
        pose2=target2
        target1,target2=self.action_to_target(actions,target1,target2)


        



        



