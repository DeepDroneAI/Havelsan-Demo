from dis import dis
import re
import numpy as np
from trajectory import Trajectory
from quadrotor import *
from airsim_update import Airsim_Updater
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry
from imuTypes import *
import rospy 
import time

class Low_Level_Updates:
    def __init__(self):
        self.updater = Airsim_Updater()
        rospy.init_node('imu_messages')
        self.imu1, self.imu2 = Imu(), Imu()
        self.odom1, self.odom2 = Odometry(), Odometry()
        self.odomSubscriber1 = 'vins_estimator1/odometry'
        self.odomSubscriber2 = 'vins_estimator2/odometry'
        self.imuSubscriber1 = '/imu_uav1'
        self.imuSubscriber2 = '/imu_uav2'
        self.imuClass = IMU()
        self.imuPub1 = rospy.Publisher('imu_uav1', Imu, queue_size=300)
        self.imuPub2 = rospy.Publisher('imu_uav2', Imu, queue_size=300)
        imu1, imu2 = Imu(), Imu()
        # IMU UPDATES
        imu1.angular_velocity.x = 0
        imu1.angular_velocity.y = 0
        imu1.angular_velocity.z = 0
        imu1.linear_acceleration.x = 0
        imu1.linear_acceleration.y = 0
        imu1.linear_acceleration.z = 0
        imu1.header.stamp = rospy.Time.now()
        imu2.angular_velocity.x = 0
        imu2.angular_velocity.y = 0
        imu2.angular_velocity.z = 0
        imu2.linear_acceleration.x = 0
        imu2.linear_acceleration.y = 0
        imu2.linear_acceleration.z = 0
        imu2.header.stamp = rospy.Time.now()
            # IMU PUBLISH
        self.imu1Publish(imu1)
        self.imu2Publish(imu2)
        input("Run subscribers")
        rospy.Subscriber(self.odomSubscriber1, Odometry, self.odom1Callback)
        rospy.Subscriber(self.odomSubscriber2, Odometry, self.odom2Callback)
        rospy.Subscriber(self.imuSubscriber1, Imu, self.imu1Callback)
        rospy.Subscriber(self.imuSubscriber2, Imu, self.imu2Callback)
        self.quad1 = Quadrotor(state0=self.__pose_to_state(self.updater.initial_poses[0]))
        self.quad2 = Quadrotor(state0=self.__pose_to_state(self.updater.initial_poses[1]))
        self.trajSelect = np.array([3, 2, 1])
        self.controller = "Backstepping_3"
        self.Tf1 = 1.0
        self.Tf2 = 1.0
        self.dtau = 1e-3
        self.bot_poses = np.zeros((2, 3))
        self.done_statu = False
        self.bot_statu_gen = np.array([1, 1])
        self.target_poses = np.array([[5, 0, 0], [5, 4, 0]])
        self.set_target_poses(self.target_poses)

        self.prev_vel1=np.zeros(3)
        self.prev_vel2=np.zeros(3)

        self.vel1=np.zeros(3)
        self.vel2=np.zeros(3)


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
        self.crash_statu = False

    def odom1Callback(self, data):
        self.odom1 = data

    def odom2Callback(self, data):
        self.odom2 = data

    def imu1Callback(self, data):
        self.imu1 = data

    def imu2Callback(self, data):
        self.imu2 = data

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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
     
        return roll_x, pitch_y, yaw_z # in radian    

    def get_quad1_state(self):
        x,y,z=[self.odom1.pose.pose.position.x,self.odom1.pose.pose.position.y,self.odom1.pose.pose.position.z]
        qx=self.odom1.pose.pose.orientation.x
        qy=self.odom1.pose.pose.orientation.y
        qz=self.odom1.pose.pose.orientation.z
        qw=self.odom1.pose.pose.orientation.w
        roll,pitch,yaw=self.euler_from_quaternion(qx,qy,qz,qw)

        ax=self.imu1.linear_acceleration.x
        ay=self.imu1.linear_acceleration.y
        az=self.imu1.linear_acceleration.z

        acc=np.array([ax,ay,az])

        self.vel1+=acc*self.dtau


        p=self.imu1.angular_velocity.x
        q=self.imu1.angular_velocity.y
        r=self.imu1.angular_velocity.z

        return [x,y,z,roll,pitch,yaw,p,q,r,self.vel1[0],self.vel1[1],self.vel1[2]]
    def get_quad2_state(self):
        x,y,z=[self.odom2.pose.pose.position.x,self.odom2.pose.pose.position.y,self.odom2.pose.pose.position.z]
        qx=self.odom2.pose.pose.orientation.x
        qy=self.odom2.pose.pose.orientation.y
        qz=self.odom2.pose.pose.orientation.z
        qw=self.odom2.pose.pose.orientation.w
        roll,pitch,yaw=self.euler_from_quaternion(qx,qy,qz,qw)

        ax=self.imu2.linear_acceleration.x
        ay=self.imu2.linear_acceleration.y
        az=self.imu2.linear_acceleration.z

        acc=np.array([ax,ay,az])

        self.vel2+=acc*self.dtau


        p=self.imu2.angular_velocity.x
        q=self.imu2.angular_velocity.y
        r=self.imu2.angular_velocity.z

        return [x,y,z,roll,pitch,yaw,p,q,r,self.vel2[0],self.vel2[1],self.vel2[2]]


    def __pose_to_state(self, pose):
        return [pose[0], pose[1], pose[2], 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def check_crash(self, pose1, pose2):
        err_sum = 0
        for i in range(3):
            err_sum += pow(pose1[i] - pose2[i], 2)

        if np.sqrt(err_sum) < 1.5:
            self.crash_statu = True

    def get_poses(self):
        poses = np.zeros((2, 3))
        for i in range(4):
            poses[i] = self.updater.get_pose(index=i + 1)
        return poses

    def get_quad1_pose(self):
        return [self.quad1.state[0], self.quad1.state[1], self.quad1.state[2]]

    def get_quad2_pose(self):
        return [self.quad2.state[0], self.quad2.state[1], self.quad2.state[2]]

    def set_target_poses(self, poses):
        all_poses = [self.updater.initial_poses[0], self.updater.initial_poses[1], poses[0], poses[1]]
        self.bot_poses[0, :] = poses[0]
        self.bot_poses[1, :] = poses[1]
        att = np.zeros((4, 3))
        self.updater.set_drone_locs(all_poses, att)

    def action_to_target(self, actions, target1, target2):
        if actions[0] == 0:
            target1[0] += 1
        elif actions[0] == 1:
            target1[0] -= 1
        elif actions[0] == 2:
            target1[1] += 1
        elif actions[0] == 3:
            target1[1] -= 1
        elif actions[0] == 4:
            target1[2] += 1
        elif actions[0] == 5:
            target1[2] -= 1

        if actions[1] == 0:
            target2[0] += 1
        elif actions[1] == 1:
            target2[0] -= 1
        elif actions[1] == 2:
            target2[1] += 1
        elif actions[1] == 3:
            target2[1] -= 1
        elif actions[1] == 4:
            target2[2] += 1
        elif actions[1] == 5:
            target2[2] -= 1

        return target1, target2

    def get_traj(self, newTraj1, newTraj2, t_current):
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

        return current_traj1, current_traj2

    def update_pr(self, traj1, traj2):
        xd1, yd1, zd1, xd_dot1, yd_dot1, zd_dot1, xd_ddot1, yd_ddot1, zd_ddot1, xd_dddot1, yd_dddot1, xd_ddddot1, yd_ddddot1, psid1, psid_dot1, psid_ddot1 = traj1
        xd2, yd2, zd2, xd_dot2, yd_dot2, zd_dot2, xd_ddot2, yd_ddot2, zd_ddot2, xd_dddot2, yd_dddot2, xd_ddddot2, yd_ddddot2, psid2, psid_dot2, psid_ddot2 = traj2

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

    def state_to_rl(self):
        p1 = self.get_quad1_pose()
        p2 = self.get_quad2_pose()
        p3 = self.bot_poses[0]
        p4 = self.bot_poses[1]
        s1 = [p1[0], p1[1], p1[2] - 6 / 5, p2[0], p2[1], p2[2], p1[0] - p3[0], p1[1] - p3[1], p1[2] - p3[2],
              p1[0] - p4[0], p1[1] - p4[1], p1[2] - p4[2]]
        s2 = [p1[0], p1[1], p1[2] - 6 / 5, 0, 0, 0, p2[0] - p3[0], p2[1] - p3[1], p2[2] - p3[2], p2[0] - p4[0],
              p2[1] - p4[1], p2[2] - p4[2]]

        return np.array([s1, s2]) * 5

    def check_collition(self):
        distance = np.zeros(4)
        poses = self.state_to_rl() / 5

        for i in range(2):
            for j in range(3):
                distance[i] += pow(poses[0][6 + i * 3 + j], 2)
                distance[i + 2] += pow(poses[1][6 + i * 3 + j], 2)

        for i in range(4):
            distance[i] = np.sqrt(distance[i])

        if distance[0] < 1.5 or distance[2] < 1.5:
            self.bot_statu_gen[0] = 0
        if distance[1] < 1.5 or distance[3] < 1.5:
            self.bot_statu_gen[1] = 0

    def step(self, actions):
        target1 = self.get_quad1_pose()
        target2 = self.get_quad2_pose()
        pose1 = target1
        pose2 = target2
        target1, target2 = self.action_to_target(actions, target1, target2)
        yaw10 = self.quad1.state[5]
        yaw20 = self.quad2.state[5]
        time_list1 = np.hstack((0., self.Tf1)).astype(float)
        waypoint_list1 = np.array([self.get_quad1_pose(), target1])
        yaw_list1 = np.hstack((yaw10, 0.)).astype(float)

        time_list2 = np.hstack((0., self.Tf2)).astype(float)
        waypoint_list2 = np.array([self.get_quad2_pose(), target2])
        yaw_list2 = np.hstack((yaw20, 0.)).astype(float)

        newTraj1 = Trajectory(self.trajSelect, self.quad1.state, time_list1, waypoint_list1, yaw_list1, v_average=5)
        newTraj2 = Trajectory(self.trajSelect, self.quad2.state, time_list2, waypoint_list2, yaw_list2, v_average=5)

        self.Tf1 = newTraj1.t_wps[1]
        self.Tf2 = newTraj2.t_wps[1]



        for i in range(100):
            time.sleep(.3)

            t_current = i * self.dtau
            traj1, traj2 = self.get_traj(t_current=t_current, newTraj1=newTraj1, newTraj2=newTraj2)

            self.quad1.simulate(self.dtau, traj1, self.controller)
            self.quad2.simulate(self.dtau, traj2, self.controller)

            print("Quad1 state:{}".format(self.get_quad1_state()))
            print("Quad2 state:{}".format(self.get_quad2_state()))

            self.update_pr(traj1=traj1, traj2=traj2)

            vel = np.sqrt(pow(self.quad1.state[6], 2) + pow(self.quad1.state[7], 2) + pow(self.quad1.state[8], 2))

            poses = np.zeros((2, 3))
            att = np.zeros((4, 3))
            poses[0, :] = np.array([self.quad1.state[0], self.quad1.state[1], self.quad1.state[2]])
            poses[1, :] = np.array([self.quad2.state[0], self.quad2.state[1], self.quad2.state[2]])
            att[0, :] = np.array([self.quad1.state[3], -self.quad1.state[4], self.quad1.state[5]])
            att[1, :] = np.array([self.quad2.state[3], -self.quad2.state[4], self.quad2.state[5]])
            self.check_collition()

            vel_cur1=np.array(self.quad1.state[6:9])
            vel_cur2=np.array(self.quad2.state[6:9])

            acc1=(vel_cur1-self.prev_vel1)/self.dtau
            acc2=(vel_cur2-self.prev_vel2)/self.dtau

            self.prev_vel1=vel_cur1
            self.prev_vel2=vel_cur2

            imu1, imu2 = Imu(), Imu()
            # IMU UPDATES
            imu1.angular_velocity.x = self.quad1.state[9]
            imu1.angular_velocity.y = self.quad1.state[10]
            imu1.angular_velocity.z = self.quad1.state[11]
            imu1.linear_acceleration.x = acc1[0]
            imu1.linear_acceleration.y = acc1[1]
            imu1.linear_acceleration.z = acc1[2]
            imu1.header.stamp = rospy.Time.now()
            imu2.angular_velocity.x = self.quad2.state[9]
            imu2.angular_velocity.y = self.quad2.state[10]
            imu2.angular_velocity.z = self.quad2.state[11]
            imu2.linear_acceleration.x = acc2[0]
            imu2.linear_acceleration.y = acc2[1]
            imu2.linear_acceleration.z = acc2[2]
            imu2.header.stamp = rospy.Time.now()
            # IMU PUBLISH
            self.imu1Publish(imu1)
            self.imu2Publish(imu2)

            if self.bot_statu_gen[0] == 0 and self.bot_statu_gen[1] == 0:
                self.done_statu = True

            if self.done_statu == True:
                return self.done_statu
            if self.bot_statu_gen[0] == 0:
                self.target_poses[0] = [0, 0, 0]
            if self.bot_statu_gen[1] == 0:
                self.target_poses[1] = [0, 0, 0]

            all_poses = np.zeros((4, 3))
            all_poses[0, :] = poses[0]
            all_poses[1, :] = poses[1]
            all_poses[2, :] = self.target_poses[0]
            all_poses[3, :] = self.target_poses[1]
            self.updater.set_drone_locs(all_poses, att)
            """self.set_target_poses(self.target_poses)
            self.updater.update_agent_pose(pose=poses)"""
        return self.done_statu

    def imu1Publish(self, imuInformation):
        self.imuPub1.publish(imuInformation)

    def imu2Publish(self, imuInformation):
        self.imuPub2.publish(imuInformation)

    def printIMUValues(self):
        print(self.imuClass.angularVelocity.toString())
        print(self.imuClass.linearAcceleration.toString())
        print(self.imuClass.orientation.toString())
        print(self.imuClass.header.toString())
        
