#! /usr/bin/env python

# Calculate route from setted two point
# using GNSS UTM cordinate date

import rospy
import time
import numpy as np
import csv
import os

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

def writer():
    global x, y
    start_x = -0.15
    start_y = 32.61
    target_x = 5.43
    target_y = 29.01
    times = 2
    dist_line = -0.5

    print "ready to save waypoint"
    print "press 1 and enter, save point 1"
    print "press 2 and enter, save point 2"
    print "press 3 and enter, save waypoint"
    print "press 0 and enter, exit"

    while not rospy.is_shutdown():
        kb = input()
    
        if kb == 0:
            print("exit")
            exit()
        elif kb == 1:    # point 1
            start_x = x
            start_y = y
            print "start_x=",x, "start_y=",y
        elif kb == 2:    # point 2
            target_x = x
            target_y = y
            print "target_x=",x, "target_y=",y
        elif kb == 3:    # backward point
            a = np.array([start_x, start_y])
            b = np.array([target_x, target_y])    
            dist = np.linalg.norm(b-a)
            target_angle = -np.arctan2(target_y - start_y, target_x - start_x)

            waypoint_x = []
            waypoint_y = []

            with open('route.csv', 'w'):
                pass
            f = open('route.csv', 'ab')
            csvWriter = csv.writer(f)

            for i in range(1,times+1):
                if i%2 == 1: 
                    waypoint_x.append(dist)
                    waypoint_x.append(0)
                    waypoint_y.append(dist_line + (i-1) * dist_line)
                    waypoint_y.append(dist_line + (i-1) * dist_line)
                else:
                    waypoint_x.append(0)
                    waypoint_x.append(dist)
                    waypoint_y.append(dist_line + (i-1) * dist_line)
                    waypoint_y.append(dist_line + (i-1) * dist_line)                    

            for i in range(len(waypoint_x)):
               listdata = []
               listdata.append(np.cos(target_angle) * waypoint_x[i] + np.sin(target_angle) * waypoint_y[i] + start_x)
               listdata.append(-np.sin(target_angle) * waypoint_x[i] + np.cos(target_angle) * waypoint_y[i] + start_y)
               if i == len(waypoint_x):
                   break
               print listdata
               csvWriter.writerow(listdata)          #row write

def ndtpose_callback(msg):
    global x,y
    x = msg.pose.position.x    # read ROS /utm topic data
    y = msg.pose.position.y
    #print(x, y)

def shutdown():
    rospy.loginfo("zigzag_writer_node was terminated")

def listener():
    rospy.init_node('route_writer_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/ndt_pose', PoseStamped, ndtpose_callback) # ROS callback function
    writer()
    rospy.spin()

if __name__ == '__main__':
    csvdir = os.path.abspath(os.path.dirname(__file__))
    os.chdir(csvdir)
    listener()
