#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

class StatusPublisher:
    def __init__(self):
        rospy.init_node("status_publisher")
        rospy.Subscriber("task_status", String, self.status_callback)
        self.status_publisher = rospy.Publisher("ui_task_status", String, queue_size = 10)
        self.status = None
        self.publisher()

    def publisher(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.status:
                self.status_publisher.publish(self.status)
            rate.sleep()
    
    def status_callback(self, msg):
        self.status = msg.data

if __name__ == '__main__':
    StatusPublisher()