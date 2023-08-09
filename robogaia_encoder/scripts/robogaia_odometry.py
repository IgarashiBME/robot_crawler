#!/usr/bin/python
import serial
import rospy
import time
import os
import sys
import numpy as np 

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import Odometry

# for encoder
PULSE_PER_REV_LEFT  = 29500.0
PULSE_PER_REV_RIGHT = 29500.0
TREAD = 0.375
PERIMETER = 0.85
INVERSE_RIGHT = -1.0
INVERSE_LEFT = -1.0
RADIUS = PERIMETER/np.pi/2.0
DIFF_THRESHOLD = 100000.0

# begin the connection to the roboteq controller
port = rospy.get_param('~port', '/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_64932343938351500132-if00')
class robogaia():
    def __init__(self):
        try:
            self.ser = serial.Serial(
            port,
            baudrate=9600,
            timeout=0.01,
            writeTimeout=0.01
        )
        except serial.serialutil.SerialException:
            rospy.logerr("robogaia encoder not found")
            sys.exit()

        rospy.loginfo("robogaia encoder was connected")
        self.pub = rospy.Publisher("/robogaia/odom", Odometry, queue_size=1)
        self.odom = Odometry()

        self.x = np.array([0, 0, self.toRadian(0.0)])
        self.u = np.array([0, 0])


    def toDegree(self, angle):
        angle = angle/np.pi *180.0
        return angle

    def toRadian(self, angle):
        angle = angle/180.0 *np.pi
        return angle

    def PItoPI(self, angle):
        while angle >= np.pi:
            angle = angle-2*np.pi
        while angle <= -np.pi:
            angle = angle+2*np.pi
        return angle

    def MotionModel(self, x, u, dt):
        F = np.eye(3)
        B = np.array([[dt*np.cos(x[2]), 0],
                      [dt*np.sin(x[2]), 0],
                      [0, dt]])
        x = np.dot(F, x) +np.dot(B, u)
        x[2] = self.PItoPI(x[2])
        return x

    def loop(self):
        rate=rospy.Rate(100)
        while not rospy.is_shutdown():
            #rate.sleep()

            try:
                resp = str(self.ser.readline()).replace("\n\r", "")
                encoderCount = resp.split(",")
                encoderCount[1]
                rightCount = float(encoderCount[0])
                leftCount = float(encoderCount[1])                
            except:
                continue

            #for i in range(len(encoderCount)):
            #    print(encoderCount[i])
            #print(rightCount, leftCount)

            try:
                last_time
            except:
                rospy.loginfo("sensor's last time was initialized")
                last_time = time.time()
                lastRightCount = rightCount
                lastLeftCount = leftCount
                continue

            # calculation
            current_time = time.time()
            dt = current_time - last_time
            diffRight = rightCount - lastRightCount
            diffLeft  = leftCount - lastLeftCount
            v_right = (float(diffRight)/PULSE_PER_REV_RIGHT) *2 *np.pi *RADIUS /dt
            v_left  = (float(diffLeft)/PULSE_PER_REV_LEFT) *2 *np.pi *RADIUS /dt

            translation_x = (v_right + v_left)/2.0
            angular_z = (v_right - v_left)/TREAD

            # update last data
            last_time = current_time
            lastRightCount = rightCount
            lastLeftCount  = leftCount

            if abs(diffRight) > DIFF_THRESHOLD or abs(diffLeft) > DIFF_THRESHOLD:
                continue

            self.u = np.array([translation_x, angular_z])
            #print("tra_x", translation_x*dt, "ang_z", angular_z/np.pi*180.0*dt)
            #print(diffRight, diffLeft)

            self.x = self.MotionModel(self.x, self.u, dt)
            #print(self.u)
            #print(self.x)
            #print(diffRight, diffLeft)
            #print("tra_x", translation_x*dt, "ang_z", angular_z/np.pi*180.0*dt)

            self.odom.header.stamp = rospy.Time.now()
            self.odom.pose.pose.position.x = self.x[0]
            self.odom.pose.pose.position.y = self.x[1]
            self.odom.pose.pose.orientation.w = self.x[2]
            self.odom.twist.twist.linear.x = self.u[0]
            self.odom.twist.twist.angular.z = self.u[1]
            self.pub.publish(self.odom)

if __name__ == "__main__":
    # start the roboteq node
    rospy.init_node('robogaia_encoder', anonymous=True)
    r = robogaia()
    r.loop()
