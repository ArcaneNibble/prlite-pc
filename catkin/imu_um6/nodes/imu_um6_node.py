#! /usr/bin/env python
import roslib; roslib.load_manifest('imu_um6')

import rospy
import math
import numpy
import tf

from um6.driver import Um6Drv
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_about_axis

class ImuUm6Node(object):

    first = True

    def __init__(self, default_port='/dev/ttyUSB0'):
        """
        @param default_port: default serial port to use for
            establishing a connection to the UM6 IMU sensor.
            This will be overridden by ~port param if 
            available.
        """
        rospy.init_node('imu_um6')

        self.port = rospy.get_param('~port', default_port)
        rospy.loginfo("serial port: %s"%(self.port))

        self.imu_data = Imu()
        # self.imu_data = Imu(header=rospy.Header(frame_id="imu_link"))
        self.imu_data = Imu(header=rospy.Header(frame_id="base_link"))
        # self.imu_data = Imu(header=rospy.Header(frame_id="base_footprint"))

        self.imu_data.orientation_covariance = [1e6, 0, 0, 
                                                0, 1e6, 0, 
                                                0, 0, 1e-6]

        self.imu_data.angular_velocity_covariance = [1e6, 0, 0,
                                                     0, 1e6, 0, 
                                                     0, 0, 1e-6]

        self.imu_data.linear_acceleration_covariance = [1e6, 0, 0, 
                                                        0, 1e6, 0, 
                                                        0, 0, 1e-6]

        self.imu_pub = rospy.Publisher('imu/data', Imu)

        # what data to pass to callback
        dataMask = (Um6Drv.DATA_QUATERNION | 
                    Um6Drv.DATA_ROLL_PITCH_YAW | 
                    Um6Drv.DATA_LINEAR_ACCEL | 
                    Um6Drv.DATA_ANGULAR_VEL)

        self.driver = Um6Drv(self.port, dataMask, self.um6_data_cb)

        # zero the gyros, reset the kalman filter, reset reference headings
        self.driver.sendCommand(Um6Drv.CMD_ZERO_GYROS, self.um6_cmd_cb)
        self.driver.sendCommand(Um6Drv.CMD_RESET_EKF, self.um6_cmd_cb)
        self.driver.sendCommand(Um6Drv.CMD_SET_MAG_REF, self.um6_cmd_cb)
        self.driver.sendCommand(Um6Drv.CMD_SET_ACCEL_REF, self.um6_cmd_cb)
        self.driver.sendConfig(Um6Drv.UM6_COMMUNICATION, Um6Drv.UM6_COMMUNICATION_DATA, self.um6_cmd_cb)

        while not rospy.is_shutdown():
            self.driver.update()

    def um6_cmd_cb(self, cmd, result):
        if (cmd == Um6Drv.UM6_COMMUNICATION):
            rospy.loginfo("Set quaternion output: %s"%(result))
        if (cmd == Um6Drv.CMD_RESET_EKF):
            rospy.loginfo("Reset EKF: %s"%(result))
        if (cmd == Um6Drv.CMD_ZERO_GYROS):
            rospy.loginfo("Zero Gyros: %s"%(result))
        if (cmd == Um6Drv.CMD_SET_MAG_REF):
            rospy.loginfo("Set Magnetometer Reference: %s"%(result))
        if (cmd == Um6Drv.CMD_SET_ACCEL_REF):
            rospy.loginfo("Set Accelerometer Reference: %s"%(result))

    def um6_data_cb(self, data):
        self.imu_data.header.stamp = rospy.Time.now()
        self.imu_data.orientation = Quaternion()

        # IMU outputs [w,x,y,z] NED, convert to [x,y,z,w] ENU
        q = [data['DATA_QUATERNION'][2],
             data['DATA_QUATERNION'][1],
            -data['DATA_QUATERNION'][3],
             data['DATA_QUATERNION'][0]]

        self.imu_data.orientation.x = q[0] 
        self.imu_data.orientation.y = q[1]
        self.imu_data.orientation.z = q[2]
        self.imu_data.orientation.w = q[3] 

        # convert to radians from degrees
        # again note NED to ENU converstion
        self.imu_data.angular_velocity.x = data['DATA_ANGULAR_VEL'][1] * (math.pi/180.0)
        self.imu_data.angular_velocity.y = data['DATA_ANGULAR_VEL'][0] * (math.pi/180.0)
        self.imu_data.angular_velocity.z = -(data['DATA_ANGULAR_VEL'][2] * (math.pi/180.0))
        # again note NED to ENU converstion
        self.imu_data.linear_acceleration.x = data['DATA_LINEAR_ACCEL'][1]
        self.imu_data.linear_acceleration.y = data['DATA_LINEAR_ACCEL'][0]
        self.imu_data.linear_acceleration.z = -(data['DATA_LINEAR_ACCEL'][2])
        
        self.imu_pub.publish(self.imu_data)

if __name__ == '__main__':
    node = ImuUm6Node()
