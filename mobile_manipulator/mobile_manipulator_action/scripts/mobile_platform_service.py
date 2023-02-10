#!/usr/bin/env python3
import rospy
from nav_msgs.msg import *
from mobile_manipulator_action.srv import MobilePlatform,MobilePlatformResponse
import rospy

class MobilePlatfromService:
    def __init__(self):
        rospy.init_node('moile_platform_server')
        s = rospy.Service('mobile_platform_service', MobilePlatform, self.server_response)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        print("Ready to add two ints.")
        rospy.spin()

    def odom_callback(self, msg):
        self.odom_vel_x = msg.twist.twist.linear.x
        self.odom_x = msg.pose.pose.position.x

    def server_response(self, req):
        status = "{\"velocity: \"" + str(self.odom_vel_x) + "," + "\"pose\": " + str(self.odom_x) + "}"
        return MobilePlatformResponse(status)

if __name__ == "__main__":
    MobilePlatfromService()