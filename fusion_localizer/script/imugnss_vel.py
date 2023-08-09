#! /usr/bin/env python

import rospy
import time
import numpy as np

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from ubx_analyzer.msg import UTMHP

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

TIMEOUT = 10.0
DIST_THRE = 0.2

class velocity_estimation():
    def __init__(self):
        rospy.init_node('velocity_node')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function
        rospy.Subscriber('/utm_hp', UTMHP, self.utm_hp)
        rospy.Subscriber('/gx5/imu/data', Imu, self.imu)

        # ROS publish function
        self.pub = rospy.Publisher('/twist_imugnss', TwistStamped, queue_size = 1)
        self.twist = TwistStamped()
        self.angular_z = 0

        #rospy.spin()

    def utm_hp(self, msg):
        self.utm_x = msg.utm_easting
        self.utm_y = msg.utm_northing
        self.utm_z = msg.heightHp

    def imu(self, msg):
        self.angular_z = msg.angular_velocity.z

    def shutdown(self):
        rospy.loginfo("fusion_imu_moving_base_node was terminated")

    def loop(self):
        rate=rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                self.utm_x
            except:
                continue

            try:
                last_utm_x
                last_utm_y
                last_time
            except:
                last_utm_x = self.utm_x
                last_utm_y = self.utm_y
                last_time = time.time()
                continue

            diff = np.sqrt((self.utm_x - last_utm_x)**2 + (self.utm_y - last_utm_y)**2)
            dt = time.time() - last_time
            linear_x = diff / dt
            #print(diff)
            #if diff < DIST_THRE:
            #    continue

            last_utm_x = self.utm_x
            last_utm_y = self.utm_y
            last_time = time.time()

            self.twist.header.stamp = rospy.Time.now()
            self.twist.header.frame_id = "base_link" 
            self.twist.twist.linear.x = linear_x
            self.twist.twist.angular.z = self.angular_z
            self.pub.publish(self.twist)
            
if __name__ == '__main__':
    v = velocity_estimation()
    v.loop()
