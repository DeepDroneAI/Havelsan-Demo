from sensor_msgs.msg import Imu
import rospy
from imuTypes import *
import numpy as np 

class TryIMU:
    def __init__(self):
        self.imu = Imu()
        self.imuClass = IMU()
        self.imuPub = rospy.Publisher('imu_messages', Imu, queue_size=300)

    def imuPublish(self, imuInformation):
        self.imuPub.publish(imuInformation)

    def printIMUValues(self):
        print(self.imuClass.angularVelocity.toString())
        print(self.imuClass.linearAcceleration.toString())
        print(self.imuClass.orientation.toString())
        print(self.imuClass.header.toString())

    def createImuValues(self):
        # IMU UPDATES
        self.imu.angular_velocity.x = np.random.random()
        self.imu.angular_velocity.y = np.random.random()
        self.imu.angular_velocity.z = np.random.random()
        self.imu.linear_acceleration.z = np.random.random()
        self.imu.linear_acceleration.y = np.random.random()
        self.imu.linear_acceleration.z = np.random.random()
        self.imu.header.stamp = rospy.Time.now()
        self.imuClass.angularVelocity.x = self.imu.angular_velocity.x
        self.imuClass.angularVelocity.y = self.imu.angular_velocity.y
        self.imuClass.angularVelocity.z = self.imu.angular_velocity.z
        self.imuClass.linearAcceleration.x = self.imu.linear_acceleration.x
        self.imuClass.linearAcceleration.y = self.imu.linear_acceleration.y
        self.imuClass.linearAcceleration.z = self.imu.linear_acceleration.z

        # IMU PUBLISH
        self.imuPublish(self.imu)

        # PRINT IMU STATES
        self.printIMUValues()


if __name__ == '__main__':
    rospy.init_node('imu_messages')
    tu = TryIMU()
    while not rospy.is_shutdown():
        tu.createImuValues()
