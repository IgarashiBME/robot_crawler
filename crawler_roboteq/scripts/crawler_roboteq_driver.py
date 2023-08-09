#!/usr/bin/python
import serial
import rospy
import time
import os
import sys
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped
MAX_SPEED = 2000
MAX_TURN = 1200

#SERIAL_MODE = "^00 01" + '\r'
#SERIAL_MODE = '\r'

# begin the connection to the roboteq controller
port = rospy.get_param('~port', '/dev/serial/by-id/usb-Roboteq_Motor_Controller_SDC2XXX-if00')
class roboteq():
    def __init__(self):
        try:
            self.ser = serial.Serial(
            port,
            baudrate=115200,
            timeout=0.01,
            writeTimeout=0.01
        )
        except serial.serialutil.SerialException:
            rospy.logerr("port not found")
            sys.exit()

        rospy.loginfo("roboteq driver was connected")
        rospy.Subscriber("cmd_vel", Twist, self.moveCallback, queue_size=1)
    # reset the connection if need be
    #if (ser.isOpen()):
    #    ser.close()
    #ser.open()

    def moveCallback(self, data):
        if (abs(data.linear.x) > 0.001 or abs(data.angular.z) > 0.001):
            speed = data.linear.x *MAX_SPEED #linear.x is value between -1 and 1 and input to wheels is between -1000 and 1000
            if speed > MAX_SPEED:
                speed = MAX_SPEED
            elif speed < -MAX_SPEED:
                speed = -MAX_SPEED
                                            #1000 would give full speed range, but setting to lower value to better control robot
            turn = (data.angular.z + 0.009)*MAX_TURN*-1
            if turn > MAX_TURN:
                turn = MAX_TURN
            elif turn < -MAX_TURN:
                turn = -MAX_TURN

            self.speed_cmd = '!G 1 ' + str(turn) + '\r'
            self.turn_cmd = '!G 2 ' + str(speed) + '\r'

            #print(self.speed_cmd, self.turn_cmd)
            self.ser.write(self.speed_cmd)
            self.ser.flush()
            self.ser.write(self.turn_cmd)
            self.ser.flush()

    def loop(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            #print self.ser.readline()
            #self.ser.write(SERIAL_MODE)
            #self.ser.flush()
            """try:
                self.ser.write(self.speed_cmd)
                self.ser.flush()
                self.ser.write(self.turn_cmd)
                self.ser.flush()
                print(self.speed_cmd, self.turn_cmd)

            except:
                pass
                #exit()"""

            try:
                self.ser.readline()

            except serial.serialutil.SerialException:
                rospy.logerr("serial exception")
                #exit()

            #self.ser.readline()
            rate.sleep()

if __name__ == "__main__":
    # start the roboteq node
    rospy.init_node('crawler_roboteq', anonymous=True)

    # start the cmd_vel subscriber
    #rospy.Subscriber("roboteq_driver/cmd", Twist, moveCallback, queue_size=1)
    r = roboteq()
    r.loop()
