import numpy as np
from trajectory import Trajectory
from quadrotor import *
from airsim_update import Airsim_Updater

class Low_Level_Updates:
    def __init__(self):
        self.updater=Airsim_Updater()
        self.quad1=Quadrotor(state0=self.__pose_to_state(self.updater.initial_poses[0]))
        self.quad2=Quadrotor(state0=self.__pose_to_state(self.updater.initial_poses[1]))
        self.trajSelect=np.array([3,2,0])
        self.controller="Backstepping_3"
        self.Tf1=1.0
        self.Tf2=1.0
        self.dtau = 1e-3

        self.xd_ddot_pr1 = 0.
        self.xd_dddot_pr1 = 0.
        self.yd_ddot_pr1 = 0.
        self.yd_dddot_pr1 = 0.
        self.psid_pr1 = 0.
        self.psid_dot_pr1 = 0.
        self.x_dot_pr1 = 0.
        self.y_dot_pr1 = 0.
        self.z_dot_pr1 = 0.

        self.xd_ddot_pr2 = 0.
        self.xd_dddot_pr2 = 0.
        self.yd_ddot_pr2 = 0.
        self.yd_dddot_pr2 = 0.
        self.psid_pr2 = 0.
        self.psid_dot_pr2 = 0.
        self.x_dot_pr2 = 0.
        self.y_dot_pr2 = 0.
        self.z_dot_pr2 = 0.
        

    def __pose_to_state(self,pose):
        return [pose[0],pose[1],pose[2],0,0,0,0,0,0,0,0,0]

    def get_poses(self):
        poses=np.zeros((2,3))
        for i in range(4):
            poses[i]=self.updater.get_pose(index=i+1)
        return poses

    def get_quad1_pose(self):
        return [self.quad1.state[0],self.quad1.state[1],self.quad1.state[2]]

    def get_quad2_pose(self):
        return [self.quad2.state[0],self.quad2.state[1],self.quad2.state[2]]

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

    def get_traj(self,newTraj1,newTraj2,t_current):
        pos_des1, vel_des1, acc_des1, euler_des1 = newTraj1.desiredState(t_current, self.dtau, self.quad1.state)
        pos_des2, vel_des2, acc_des2, euler_des2 = newTraj2.desiredState(t_current, self.dtau, self.quad2.state)

        xd1, yd1, zd1 = pos_des1[0], pos_des1[1], pos_des1[2]
        xd_dot1, yd_dot1, zd_dot1 = vel_des1[0], vel_des1[1], vel_des1[2]
        xd_ddot1, yd_ddot1, zd_ddot1 = acc_des1[0], acc_des1[1], acc_des1[2]

        xd_dddot1 = (xd_ddot1 - self.xd_ddot_pr1) / self.dtau
        yd_dddot1 = (yd_ddot1 - self.yd_ddot_pr1) / self.dtau
        xd_ddddot1 = (xd_dddot1 - self.xd_dddot_pr1) / self.dtau
        yd_ddddot1 = (yd_dddot1 - self.yd_dddot_pr1) / self.dtau

        psid1 = euler_des1[2]

        psid_dot1 = (psid1 - self.psid_pr1) / self.dtau
        psid_ddot1 = (psid_dot1 - self.psid_dot_pr1) / self.dtau

        current_traj1 = [xd1, yd1, zd1, xd_dot1, yd_dot1, zd_dot1, xd_ddot1, yd_ddot1, zd_ddot1,
                         xd_dddot1, yd_dddot1, xd_ddddot1, yd_ddddot1,
                         psid1, psid_dot1, psid_ddot1]



        xd2, yd2, zd2 = pos_des2[0], pos_des2[1], pos_des2[2]
        xd_dot2, yd_dot2, zd_dot2 = vel_des2[0], vel_des2[1], vel_des2[2]
        xd_ddot2, yd_ddot2, zd_ddot2 = acc_des2[0], acc_des2[1], acc_des2[2]

        xd_dddot2 = (xd_ddot2 - self.xd_ddot_pr2) / self.dtau
        yd_dddot2 = (yd_ddot2 - self.yd_ddot_pr2) / self.dtau
        xd_ddddot2 = (xd_dddot2 - self.xd_dddot_pr2) / self.dtau
        yd_ddddot2 = (yd_dddot2 - self.yd_dddot_pr2) / self.dtau

        psid2 = euler_des2[2]

        psid_dot2 = (psid2 - self.psid_pr2) / self.dtau
        psid_ddot2 = (psid_dot2 - self.psid_dot_pr2) / self.dtau

        current_traj2 = [xd2, yd2, zd2, xd_dot2, yd_dot2, zd_dot2, xd_ddot2, yd_ddot2, zd_ddot2,
                         xd_dddot2, yd_dddot2, xd_ddddot2, yd_ddddot2,
                         psid2, psid_dot2, psid_ddot2]

        return current_traj1,current_traj2

    def update_pr(self,traj1,traj2):
            xd1, yd1, zd1, xd_dot1, yd_dot1, zd_dot1, xd_ddot1, yd_ddot1, zd_ddot1,xd_dddot1, yd_dddot1, xd_ddddot1, yd_ddddot1, psid1, psid_dot1, psid_ddot1=traj1
            xd2, yd2, zd2, xd_dot2, yd_dot2, zd_dot2, xd_ddot2, yd_ddot2, zd_ddot2,xd_dddot2, yd_dddot2, xd_ddddot2, yd_ddddot2, psid2, psid_dot2, psid_ddot2=traj2

            self.xd_ddot_pr1 = xd_ddot1
            self.yd_ddot_pr1 = yd_ddot1
            self.xd_dddot_pr1 = xd_dddot1
            self.yd_dddot_pr1 = yd_dddot1
            self.psid_pr1 = psid1
            self.psid_dot_pr1 = psid_dot1

            self.x_dot_pr1 = self.quad1.state[6]
            self.y_dot_pr1 = self.quad1.state[7]
            self.z_dot_pr1 = self.quad1.state[8]

            self.xd_ddot_pr2 = xd_ddot2
            self.yd_ddot_pr2 = yd_ddot2
            self.xd_dddot_pr2 = xd_dddot2
            self.yd_dddot_pr2 = yd_dddot2
            self.psid_pr2 = psid2
            self.psid_dot_pr2 = psid_dot2

            self.x_dot_pr2 = self.quad2.state[6]
            self.y_dot_pr2 = self.quad2.state[7]
            self.z_dot_pr2 = self.quad2.state[8]


    def step(self,actions):
        target1=self.get_quad1_pose()
        target2=self.get_quad2_pose()
        pose1=target1
        pose2=target2
        target1,target2=self.action_to_target(actions,target1,target2)
        yaw10 = self.quad1.state[5]
        yaw20 = self.quad2.state[5]
        time_list1 = np.hstack((0., self.Tf1)).astype(float)
        waypoint_list1 = np.vstack((pose1, target1)).astype(float)
        yaw_list1 = np.hstack((yaw10, 0.)).astype(float)


        time_list2 = np.hstack((0., self.Tf2)).astype(float)
        waypoint_list2 = np.vstack((pose2, target2)).astype(float)
        yaw_list2 = np.hstack((yaw20, 0.)).astype(float)

        newTraj1 = Trajectory(self.trajSelect, self.quad1.state, time_list1, waypoint_list1, yaw_list1, v_average=1) 
        newTraj2 = Trajectory(self.trajSelect, self.quad2.state, time_list2, waypoint_list2, yaw_list2, v_average=1) 

        self.Tf1 = newTraj1.t_wps[1]
        self.Tf2 = newTraj2.t_wps[1]

        for i in range(100):
            t_current=i*self.dtau
            traj1,traj2=self.get_traj(t_current=t_current,newTraj1=newTraj1,newTraj2=newTraj2)

            self.quad1.simulate(self.dtau, traj1,self.controller)
            self.quad2.simulate(self.dtau, traj2,self.controller)

            self.update_pr(traj1=traj1,traj2=traj2)

            poses=np.zeros((2,3))
            poses[0,:]=np.array([self.quad1.state[0], self.quad1.state[1], self.quad1.state[2]])
            poses[1,:]=np.array([self.quad2.state[0], self.quad2.state[1], self.quad2.state[2]])


            self.updater.update_agent_pose(pose=poses)






        


