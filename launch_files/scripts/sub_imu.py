#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

def imu(msg):
    qx = msg.orientation.x
    qy = msg.orientation.y
    qz = msg.orientation.z
    qw = msg.orientation.w

    euler = euler_from_quaternion([qx, qy, qz, qw])
    print("roll", euler[0]/3.1415*180, "pitch", euler[1]/3.1415*180, "yaw", euler[2]/3.1415*180)

def shutdown():
    rospy.loginfo("imu_republish_node was terminated")

def listener():
    rospy.init_node('imu_republish_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/gx5/imu/data', Imu, imu) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
