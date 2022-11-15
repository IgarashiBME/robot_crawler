#! /usr/bin/env python

import rospy
import time
import numpy as np

from geometry_msgs.msg import PoseStamped
from ubx_analyzer.msg import UTMHP

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

TIMEOUT = 10.0
DIST_THRE = 0.2

class gnss():
    def __init__(self):
        rospy.init_node('gnss_odom_node')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function
        rospy.Subscriber('/utm_hp', UTMHP, self.utm_hp)
        # ROS publish function
        self.pub = rospy.Publisher('/gnss_pose', PoseStamped, queue_size = 1)
        self.gnss_odom = PoseStamped()

        #rospy.spin()

    def utm_hp(self, msg):
        self.utm_x = msg.utm_easting - 374482.858
        self.utm_y = msg.utm_northing - 3949920.31
        self.utm_z = msg.heightHp - 85.794

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
            except:
                last_utm_x = self.utm_x
                last_utm_y = self.utm_y
                continue

            diff = np.sqrt((self.utm_x - last_utm_x)**2 + (self.utm_y - last_utm_y)**2)
            #print(diff)
            if diff < DIST_THRE:
                continue

            #print(self.utm_x, self.utm_y)
            #print(last_utm_x, last_utm_y)
            yaw = np.arctan2((self.utm_y - last_utm_y), (self.utm_x - last_utm_x))
            last_utm_x = self.utm_x
            last_utm_y = self.utm_y

            quat = quaternion_from_euler(0, 0, yaw)
            #print(yaw/np.pi*180)

            self.gnss_odom.header.stamp = rospy.Time.now()
            self.gnss_odom.header.frame_id = "map"            
            self.gnss_odom.pose.position.x = self.utm_x
            self.gnss_odom.pose.position.y = self.utm_y
            self.gnss_odom.pose.position.z = self.utm_z
            self.gnss_odom.pose.orientation.x = quat[0]
            self.gnss_odom.pose.orientation.y = quat[1]
            self.gnss_odom.pose.orientation.z = quat[2]
            self.gnss_odom.pose.orientation.w = quat[3]
            self.pub.publish(self.gnss_odom)            

if __name__ == '__main__':
    g = gnss()
    g.loop()
