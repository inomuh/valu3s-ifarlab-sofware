#!/usr/bin/env python3

import numpy as np
import time

import rospy
from std_msgs.msg import Int8, String

class Emergency:
    def __init__(self):
        rospy.init_node("odom_combined")
        # self.emg_publisher = rospy.Publisher("emg", Int8, queue_size=10)
        self.light_curtain_publisher = rospy.Publisher("robot_rgb", String, queue_size=10)
        # rospy.Subscriber("emg", Int8, self.emg_callback)
        rospy.Subscriber("ui_emg", Int8, self.ui_emg_callback)
        self.ui_emg = Int8()
        self.ui_emg.data = 0
        self.main()
    
    def main(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # self.emg_publisher.publish(self.ui_emg)
            rate.sleep()

    def emg_callback(self, msg):
        self.general_emg = msg.data

    def ui_emg_callback(self, msg):
        self.ui_emg.data = msg.data
        if self.ui_emg.data == 0:
            self.light_curtain_publisher.publish("{\"ID\":44}")

if __name__ == "__main__":
    Emergency()






