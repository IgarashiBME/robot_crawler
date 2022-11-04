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

ENCODER_READ_CH1 = "?CR 1\r"
ENCODER_READ_CH2 = "?CR 2\r"

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
            #self.ser.flush()
            self.ser.write(self.turn_cmd)
            #self.ser.flush()

    def loop(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            try:
                self.ser.readline()
                self.ser.write(ENCODER_READ_CH1)
                resp_ch1 = self.ser.readline()
                self.ser.write(ENCODER_READ_CH2)
                resp_ch2 = self.ser.readline()

            except serial.serialutil.SerialException:
                rospy.logerr("serial exception")
                continue

            for i in range(len(resp_ch1)):
                if resp_ch1[0+i:1+i] == "C" and resp_ch1[1+i:2+i] == "R" and resp_ch1[2+i:3+i] == "=":

                    j = 1
                    ch1 = resp_ch1[2+i+j:3+i+j]
                    while True:
                        if resp_ch1[3+i+j:4+i+j] == "\r":
                            #print("CH1", ch1)
                            break

                        ch1 = ch1 + resp_ch1[3+i+j:4+i+j]
                        j = j+1

            for i in range(len(resp_ch2)):
                if resp_ch2[0+i:1+i] == "C" and resp_ch2[1+i:2+i] == "R" and resp_ch2[2+i:3+i] == "=":

                    j = 1
                    ch2 = resp_ch2[2+i+j:3+i+j]
                    while True:
                        if resp_ch2[3+i+j:4+i+j] == "\r":
                            #print("CH2", ch2)
                            break

                        ch2 = ch2 + resp_ch2[3+i+j:4+i+j]
                        j = j+1
            print("CH1", ch1, "CH2", ch2)

if __name__ == "__main__":
    # start the roboteq node
    rospy.init_node('crawler_roboteq', anonymous=True)

    # start the cmd_vel subscriber
    #rospy.Subscriber("roboteq_driver/cmd", Twist, moveCallback, queue_size=1)
    r = roboteq()
    r.loop()
