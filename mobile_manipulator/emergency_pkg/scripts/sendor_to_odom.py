#!/usr/bin/env python3

import numpy as np
import time
import json

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class SensorOdom:
    def __init__(self):
        rospy.init_node("odom_combined")
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("low_laser", String, self.ir_callback)
        self.ir_origin = 2.4988601207733154
        self.main()
    
    def main(self):
        rate = rospy.Rate(5)
        self.ir_position = None
        self.odom = [0,0,0,0,0]
        ir_vel = 0
        first_pose_flag = False
        time_diff = 0

        while not rospy.is_shutdown():
            if self.ir_position is not None:
                if not first_pose_flag:
                    pre_pose = self.ir_position
                    pre_time = time.time()
                    first_pose_flag = True
                pose_diff = self.ir_position - pre_pose
                try:
                    ir_vel = pose_diff / (time.time() - pre_time)
                    print("vel",ir_vel, time.time() - pre_time)
                except:
                    pass
                pre_pose = self.ir_position
                pre_time = time.time()
            rate.sleep()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x,
                                            orientation.y,
                                            orientation.z,
                                            orientation.w])
        twist = msg.twist.twist
        self.odom = [position.x,
                    position.y,
                    yaw,
                    twist.linear.x,
                    twist.angular.z]
        # print(self.odom)
    
    def ir_callback(self, msg):
        data = json.loads(msg.data)
        self.ir_position = float(data["Distance_MM"]) / 1000
        print("pose",self.ir_position)

        # self.ir_position = self.ir_origin - data

if __name__ == "__main__":
    SensorOdom()






