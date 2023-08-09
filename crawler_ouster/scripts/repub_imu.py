#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

imu_raw = Imu()
pub_imu = rospy.Publisher('/repub_imu', Imu, queue_size=1)

def imu(msg):
    imu_raw = msg
    imu_raw.header.stamp = rospy.Time.now()
    pub_imu.publish(imu_raw)

def shutdown():
    rospy.loginfo("imu_republish_node was terminated")

def listener():
    rospy.init_node('imu_republish_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/os_cloud_node/imu', Imu, imu) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
