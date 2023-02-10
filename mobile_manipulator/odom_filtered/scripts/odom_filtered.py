#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class OdomFiltered():
    def __init__(self):
        rospy.init_node("odom_filtered")
        rospy.Subscriber("ota_ir", Range, self.sensor_callback)
        self.sensor_pos_publisher = rospy.Publisher("sensor_position", Float32, queue_size=10)
        self.ir_origin = 2.4988601207733154

    def execute(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print(self.ir_position)
            rate.sleep()
    
    def sensor_callback(self, msg):
        data = msg.range
        self.ir_position = self.ir_origin - data
        self.sensor_pos_publisher.publish(self.ir_position)

if __name__ == '__main__':
    OdomFiltered()