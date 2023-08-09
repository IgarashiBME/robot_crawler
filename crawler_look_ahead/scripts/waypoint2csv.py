#! /usr/bin/python
# coding:utf-8

import rospy
import csv
import os

import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

# frequency [Hz]
FREQUENCY = 1

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
        while not rospy.is_shutdown():
            try:
                t = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rate.sleep()
                continue

            rover_pos  = t.transform.translation
            rover_quat = t.transform.rotation

            print("x=",rover_pos.x, "y=",rover_pos.y)
            kb = raw_input()    
            if kb == 0:
                print("exit")
                exit()
            elif kb == "s":    # save waypoint
                f = open('route.csv', 'ab')
                csvWriter = csv.writer(f)
                data = []
                data.append(rover_pos.x)
                data.append(rover_pos.y)
                csvWriter.writerow(data)
                print("save: ","x=",rover_pos.x, "y=",rover_pos.y)
                f.close()

            rate.sleep()
    
if __name__ == '__main__':
    w = waypoint2csv()
    w.loop()
