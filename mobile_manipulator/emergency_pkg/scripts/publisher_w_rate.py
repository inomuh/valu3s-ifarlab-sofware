#!/usr/bin/env python3

import numpy as np
import time

import rospy
from std_msgs.msg import Int8, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class RatePublisher:
    def __init__(self):
        rospy.init_node("odom_combined")
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.robot_status = rospy.Publisher("mobile_status", String, queue_size=10)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("emg", Int8, self.emg_callback)

        self.odom_x = 0
        self.vx = 0
        self.cmd_vx = 0
        self.emg_data = 0

        self.main()
    
    def main(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if abs(self.vx) > 0.01:
                data = "Moving"
            else:
                data = "Idle"

            if self.odom_x > 2.10:
                data = "Max_limit"
            elif self.odom_x < 0.02:
                data = "Min_limit"

            if self.emg_data:
                data = "Alert"
            self.robot_status.publish(data)
            rate.sleep()

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.vx = msg.twist.twist.linear.x

    def emg_callback(self, msg):
        self.emg_data = msg.data
    
    def cmd_callback(self, msg):
        self.cmd_vx = msg.linear.x

if __name__ == "__main__":
    RobotStatus()
