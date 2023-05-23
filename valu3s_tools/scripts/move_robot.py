#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class DigitalTwinOdom():
    def __init__(self):
        rospy.init_node("digital_twin")
        rospy.Subscriber('/odom', Odometry, self.callback)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy(
                '/gazebo/set_model_state', SetModelState)
        self.pre_pose = None
        rospy.spin()

    def callback(self, msg):
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        pose_orientation = msg.pose.pose.orientation
        # print(pose_x)
        state_msg = ModelState()
        state_msg.model_name = "robot"
        state_msg.pose.position.x = pose_x
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.162
        state_msg.pose.orientation.w = 1
        try:
            if self.pre_pose != None:
                if abs(abs(self.pre_pose) - abs(pose_x)) > 0.01:
                    # print(abs(abs(self.pre_pose) - abs(pose_x)))
                    self.set_state(state_msg)
        except Exception as e:
            print("fail set product", e)
        self.pre_pose = pose_x

if __name__ == '__main__':
    DigitalTwinOdom()

