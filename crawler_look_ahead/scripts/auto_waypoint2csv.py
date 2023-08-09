#! /usr/bin/python
# coding:utf-8

import csv
import os
import numpy as np

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

# frequency [Hz]
FREQUENCY = 1

# Constant
SPACING = 0.2 # Unit is meter

class waypoint2csv():
    def __init__(self):
        rospy.init_node('waypoint2csv')
        rospy.on_shutdown(self.shutdown)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def shutdown(self):
        print "shutdown"

    def loop(self):
        rate=rospy.Rate(FREQUENCY)
        pre_pos_x = None
        pre_pos_y = None
        while not rospy.is_shutdown():
            try:
                t = self.tfBuffer.lookup_transform('t265_odom_frame', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rate.sleep()
                continue

            rover_pos  = t.transform.translation
            rover_quat = t.transform.rotation

            if pre_pos_x == None and pre_pos_y == None:
                f = open('route.csv', 'ab')
                csvWriter = csv.writer(f)
                data = []
                data.append(rover_pos.x)
                data.append(rover_pos.y)
                csvWriter.writerow(data)
                print("x=",rover_pos.x, "y=",rover_pos.y)

                pre_pos_x = rover_pos.x
                pre_pos_y = rover_pos.y
                continue

            a = np.array([pre_pos_x, pre_pos_y])
            b = np.array([rover_pos.x, rover_pos.y])

            # Distance between past waypoint and current waypoint
            d = np.linalg.norm(a-b)
            print(d)
            if abs(d) > SPACING:
                data = []
                data.append(rover_pos.x)
                data.append(rover_pos.y)
                csvWriter.writerow(data)

                print("x=",rover_pos.x, "y=",rover_pos.y)

                pre_pos_x = rover_pos.x
                pre_pos_y = rover_pos.y

            rate.sleep()
        f.close()
    
if __name__ == '__main__':
    w = waypoint2csv()
    w.loop()
