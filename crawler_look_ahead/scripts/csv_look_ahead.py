#! /usr/bin/python
# coding:utf-8

import rospy
import numpy as np
import csv
import time

import load_waypoint

import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply

# ros custom message
from crawler_look_ahead.msg import Lookahead_Log

LOOK_AHEAD_DIST = 1.3  # look-ahead distance [meter]
SPACING = 0.3       # distance between lines 
x_tolerance = 0.1  # [meter]
YAW_TOLERANCE = 30.0 # [Degree]

YAW_TOLERANCE_ONSTART = 5.0 # [Degree]

I_CONTROL_DIST = 0.1 # [meter], refer to cross_track_error 
MAX_PIVOT_COUNT = 1

# translation value
FORWARD_CONST = 1
BACKWARD_CONST = -1

# AJK
TRANSLATION_NEUTRAL = 512     # neutral value
STEERING_NEUTRAL = 512        # neutral value
RIGHT_PIVOT = 332
LEFT_PIVOT = 692
FB_OPTIMUM = 220
LR_OPTIMUM = 60

# for simulator or test vehicle
CMD_LINEAR_OPT = 0.2
CMD_ANGULAR_RIGHT = -0.2
CMD_ANGULAR_LEFT = 0.2
CMD_ANGULAR_K = 0.2
CMD_ANGULAR_LIMIT = 0.2

# gain
KP = 0.03
KI = 1.0
KD = 0.1

# frequency [Hz]
FREQUENCY = 10

class look_ahead():
    def __init__(self):
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_goal = []
        self.x = 0
        self.y = 0
        self.yaw = np.pi/2
        self.pre_steering_ang = 0

        rospy.init_node('look_ahead_following')
        rospy.on_shutdown(self.shutdown)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.cmdvel = Twist()

        self.lookahead_log_pub = rospy.Publisher('/lookahead_log', Lookahead_Log, queue_size = 1)
        self.lookahead_log = Lookahead_Log()

    def cmdvel_publisher(self, steering_ang, translation, pi):
        if abs(steering_ang) > YAW_TOLERANCE:
            if steering_ang >= 0:
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = CMD_ANGULAR_LEFT
            else:
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = CMD_ANGULAR_RIGHT
        else:
            self.cmdvel.linear.x = CMD_LINEAR_OPT *translation
            self.cmdvel.angular.z = pi *CMD_ANGULAR_K

            # Angular limit
            if self.cmdvel.angular.z > CMD_ANGULAR_LIMIT:
                self.cmdvel.angular.z = CMD_ANGULAR_LIMIT
            elif self.cmdvel.angular.z < -CMD_ANGULAR_LIMIT:
                self.cmdvel.angular.z = -CMD_ANGULAR_LIMIT

        self.cmdvel_pub.publish(self.cmdvel)
        return self.cmdvel.linear.x, self.cmdvel.angular.z

    def shutdown(self):
        print "shutdown"

    def loop(self):
        seq = 1
        self.last_steering_ang = 0
        rate=rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # if a specific variable is exists, the proceeds
            try:
                t = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rate.sleep()
                continue

            rover_pos  = t.transform.translation
            rover_quat = t.transform.rotation
 
            # waypoint with xy coordinate origin adjust
            if seq == 0:
                wp_x_adj = self.waypoint_x[seq] - rover_pos.x
                wp_y_adj = self.waypoint_y[seq] - rover_pos.y
                own_x_adj = 0
                own_y_adj = 0
            else:
                wp_x_adj = self.waypoint_x[seq] - self.waypoint_x[seq-1]
                wp_y_adj = self.waypoint_y[seq] - self.waypoint_y[seq-1]
                own_x_adj = rover_pos.x - self.waypoint_x[seq-1]
                own_y_adj = rover_pos.y - self.waypoint_y[seq-1]

            # coordinate transformation of waypoint
            tf_angle = np.arctan2(wp_y_adj, wp_x_adj)
            wp_x_tf = wp_x_adj*np.cos(-tf_angle) - wp_y_adj*np.sin(-tf_angle)
            wp_y_tf = wp_x_adj*np.sin(-tf_angle) + wp_y_adj*np.cos(-tf_angle)

            # coordinate transformation of own position
            own_x_tf = own_x_adj*np.cos(-tf_angle) - own_y_adj*np.sin(-tf_angle)
            own_y_tf = own_x_adj*np.sin(-tf_angle) + own_y_adj*np.cos(-tf_angle)


            # coordinate transformation of own position
            tf_q = quaternion_from_euler(0, 0, tf_angle)
            front_q_tf = quaternion_multiply((rover_quat.x, rover_quat.y, rover_quat.z, rover_quat.w), 
                                             (     tf_q[0],      tf_q[1],      tf_q[2],     -tf_q[3]))

            # inverted
            rear_q_tf = np.empty(4)
            rear_q_tf[0] = front_q_tf[0]
            rear_q_tf[1] = front_q_tf[1]
            rear_q_tf[2] = front_q_tf[3]
            rear_q_tf[3] = -front_q_tf[2]

            # calculate the target-angle(bearing) using look-ahead distance
            bearing = np.arctan2(-own_y_tf, LOOK_AHEAD_DIST)
            bearing_q = quaternion_from_euler(0, 0, bearing)

            # calculate the minimal yaw error, and decide the forward or backward
            front_steering_q = quaternion_multiply(( bearing_q[0],  bearing_q[1],  bearing_q[2],  bearing_q[3]),
                                                   (front_q_tf[0], front_q_tf[1], front_q_tf[2], -front_q_tf[3]))
            rear_steering_q = quaternion_multiply((bearing_q[0], bearing_q[1], bearing_q[2], bearing_q[3]),
                                                  (rear_q_tf[0], rear_q_tf[1], rear_q_tf[2], -rear_q_tf[3]))

            front_steering_ang = euler_from_quaternion(front_steering_q)[2]/np.pi *180
            rear_steering_ang = euler_from_quaternion(rear_steering_q)[2]/np.pi *180

            if abs(front_steering_ang) >= abs(rear_steering_ang):
                steering_ang = rear_steering_ang
                translation = BACKWARD_CONST
            elif abs(front_steering_ang) < abs(rear_steering_ang):
                steering_ang = front_steering_ang
                translation = FORWARD_CONST
            steering_ang = front_steering_ang
            translation = FORWARD_CONST

            # calculate the steering_value
            p = KP *steering_ang
            i = KI *own_y_tf
            d = KD * (self.last_steering_ang - steering_ang)
            self.last_steering_ang = steering_ang

            pi_value = p - d
            if abs(own_y_tf) < I_CONTROL_DIST:
                pi_value = p - i

            linear_x, angular_z = self.cmdvel_publisher(steering_ang, translation, pi_value)

            # AJK steering.
            """ajk_steering = STEERING_NEUTRAL +LR_OPTIMUM *pi_value
            if translation < 0:
                ajk_steering = STEERING_NEUTRAL -LR_OPTIMUM *pi_value 
            ajk_translation = TRANSLATION_NEUTRAL +FB_OPTIMUM *translation

            # Restriction of ajk_steering
            if ajk_steering > TRANSLATION_NEUTRAL + LR_OPTIMUM:
                   ajk_steering = TRANSLATION_NEUTRAL + LR_OPTIMUM
            elif ajk_steering < TRANSLATION_NEUTRAL - LR_OPTIMUM:
                  ajk_steering = TRANSLATION_NEUTRAL - LR_OPTIMUM"""

            # If the yaw error is large, start pivot turn.
            # When the path has just changed(self.bool_start_point is True), start to larger pivot turn.
            #
            # An upper limit is provided to avoid situation where only a pivot turn is made.
            """if abs(steering_ang) > YAW_TOLERANCE and self.bool_start_point == False:
                if steering_ang >= 0:
                    self.ajk_value.stamp = rospy.Time.now()
                    self.ajk_value.translation = TRANSLATION_NEUTRAL
                    self.ajk_value.steering = LEFT_PIVOT
                else:
                    self.ajk_value.stamp = rospy.Time.now()
                    self.ajk_value.translation = TRANSLATION_NEUTRAL
                    self.ajk_value.steering = RIGHT_PIVOT
            elif abs(steering_ang) > YAW_TOLERANCE_ONSTART and self.bool_start_point == True:
                if steering_ang >= 0:
                    self.ajk_value.stamp = rospy.Time.now()
                    self.ajk_value.translation = TRANSLATION_NEUTRAL
                    self.ajk_value.steering = LEFT_PIVOT                    
                else:
                    self.ajk_value.stamp = rospy.Time.now()
                    self.ajk_value.translation = TRANSLATION_NEUTRAL
                    self.ajk_value.steering = RIGHT_PIVOT
                if pivot_count > MAX_PIVOT_COUNT:
                    pivot_count = 0
                    self.bool_start_point == False
                else:
                    pivot_count = pivot_count + 1                   
            else:
                self.ajk_value.stamp = rospy.Time.now()
                self.ajk_value.translation = ajk_translation
                self.ajk_value.steering = ajk_steering
            self.ajk_pub.publish(self.ajk_value)"""

            #print wp_x_adj, wp_y_adj, tf_angle/np.pi*180

            # publish autonomous log
            self.lookahead_log.stamp = rospy.Time.now()
            self.lookahead_log.waypoint_seq = seq
            self.lookahead_log.waypoint_start_x = self.waypoint_x[seq-1]
            self.lookahead_log.waypoint_start_y = self.waypoint_y[seq-1]
            self.lookahead_log.waypoint_end_x = self.waypoint_x[seq]
            self.lookahead_log.waypoint_end_y = self.waypoint_y[seq]
            self.lookahead_log.own_x = rover_pos.x
            self.lookahead_log.own_y = rover_pos.y
            self.lookahead_log.own_yaw = euler_from_quaternion((rover_quat.x, 
                                                                rover_quat.y,
                                                                rover_quat.z,
                                                                rover_quat.w))[2]/np.pi * 180.0
            self.lookahead_log.tf_waypoint_x = wp_x_tf
            self.lookahead_log.tf_waypoint_y = wp_y_tf
            self.lookahead_log.tf_own_x = own_x_tf
            self.lookahead_log.tf_own_y = own_y_tf
            self.lookahead_log.cross_track_error = -own_y_tf
            self.lookahead_log.Kp = KP
            self.lookahead_log.Ki = KI
            self.lookahead_log.Kd = KD
            self.lookahead_log.look_ahead_dist = LOOK_AHEAD_DIST
            self.lookahead_log.i_control_dist = I_CONTROL_DIST
            self.lookahead_log.p = p
            self.lookahead_log.i = i
            self.lookahead_log.d = d
            self.lookahead_log.steering_ang = steering_ang
            self.lookahead_log.linear_x = linear_x
            self.lookahead_log.angular_z = angular_z
            self.lookahead_log_pub.publish(self.lookahead_log)

            # distance from target
            u = np.array([wp_x_tf, wp_y_tf])
            v = np.array([own_x_tf, own_y_tf])
            d = np.cross(u, v) / (np.linalg.norm(u) + 0.0000001)

            #print("sequence:", seq)
            #print("transform_wx:", wp_x_tf, "transform_wy:", wp_y_tf)
            #print("transform_own_x:", own_x_tf, "transform_own_y:", own_y_tf)
            #print("target_dist:", d)
            #print("steering_angle:",steering_ang)
            #print("linear:",linear_x, "angular:",angular_z)
            #print("\n")
            #print "cross_track_error:", d
            #print front_steering_ang, rear_steering_ang
            #print steering_ang, pd_value
            #print self.ajk_value.translation, self.ajk_value.steering

            # when reaching the look-ahead distance, read the next waypoint.
            if (wp_x_tf - own_x_tf) < x_tolerance:
                pre_wp_x = self.waypoint_x[seq]
                pre_wp_y = self.waypoint_y[seq]
                seq = seq + 1
                try:
                    a = np.array([pre_wp_x, pre_wp_y])
                    b = np.array([self.waypoint_x[seq], self.waypoint_y[seq]])

                    if np.linalg.norm(a-b) < SPACING:
                        seq = seq + 1
                except IndexError:
                    pass

            if seq >= len(self.waypoint_x):
                """self.ajk_value.stamp = rospy.Time.now()
                self.ajk_value.translation = TRANSLATION_NEUTRAL
                self.ajk_value.steering = STEERING_NEUTRAL
                self.ajk_pub.publish(self.ajk_value)"""

                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = 0
                self.cmdvel_pub.publish(self.cmdvel)
                seq = 1
                print "mission_end"
                #break
            #print
            rate.sleep()

    # load waypoint list
    def load_waypoint(self):
        self.waypoint_x, self.waypoint_y = load_waypoint.load_csv()
        #print self.waypoint_x, self.waypoint_y
    
if __name__ == '__main__':
    l = look_ahead()
    l.load_waypoint()
    l.loop()
