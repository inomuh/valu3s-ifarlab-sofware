#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import xml.etree.ElementTree as ET
import json
import math

class DigitalTwinOdt():
    def __init__(self):
        rospy.init_node("digital_twin_odt")
        self.state_msg = ModelState()
        self.cylinder_chkbx = False
        self.joint_chkbx = False
        self.model_move = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size = 50)
        rospy.Subscriber("odt_json", String, self.odt_callback)
        rospy.Subscriber("oht_human_info", String, self.oht_callback)
        rospy.Subscriber("mam_chkbx", String, self.mam_chkbx_callback)
        self.pose = Pose()
        self.pose.position.x = 100
        self.pose.position.y = 100
        self.pose.position.z = 100

        rospy.spin()

    def oht_callback(self, msg):
        json_data = json.loads(msg.data)
        self.state_msg.pose.position.x = json_data["x"]
        self.state_msg.pose.position.y = json_data["y"]
        self.state_msg.pose.position.z = 0
        self.state_msg.model_name = "person_walking"
        self.model_move.publish(self.state_msg)

    def odt_callback(self, msg):
        pose = Pose()
        data = json.loads(msg.data)
        if self.cylinder_chkbx:
            self.move_cylinder("cylinder1", data["cylinder0"])
            self.move_cylinder("cylinder2", data["cylinder1"])
            self.move_cylinder("cylinder4", data["cylinder2"])
            self.move_cylinder("cylinder5", data["cylinder3"])

        else:
            self.state_msg.pose = self.pose
            self.state_msg.model_name = "cylinder1"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "cylinder2"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "cylinder4"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "cylinder5"
            self.model_move.publish(self.state_msg)

        if self.joint_chkbx:
            a = np.array([float(data["dist0"]["fin_pose"]["x"]), float(data["dist0"]["fin_pose"]["y"]) -0.26, float(data["dist0"]["fin_pose"]["z"])])
            b = np.array([float(data["dist0"]["strt_pose"]["x"]), float(data["dist0"]["strt_pose"]["y"]) -0.26, float(data["dist0"]["strt_pose"]["z"])])
            orientation = self.calculate_rpy(a, b)
            item_pose = Pose(Point(x=(a[0]+b[0])/2, y=(a[1]+b[1])/2, z=(a[2]+b[2])/2),   orientation)
            self.state_msg.model_name = "dist0"
            self.state_msg.pose = item_pose
            self.model_move.publish(self.state_msg)

            a = np.array([float(data["dist1"]["fin_pose"]["x"]), float(data["dist1"]["fin_pose"]["y"]) -0.26, float(data["dist1"]["fin_pose"]["z"])])
            b = np.array([float(data["dist1"]["strt_pose"]["x"]), float(data["dist1"]["strt_pose"]["y"]) -0.26, float(data["dist1"]["strt_pose"]["z"])])
            orientation = self.calculate_rpy(a, b)
            item_pose = Pose(Point(x=(a[0]+b[0])/2, y=(a[1]+b[1])/2, z=(a[2]+b[2])/2),   orientation)
            self.state_msg.model_name = "dist1"
            self.state_msg.pose = item_pose
            self.model_move.publish(self.state_msg)

            a = np.array([float(data["dist2"]["fin_pose"]["x"]), float(data["dist2"]["fin_pose"]["y"]) -0.26, float(data["dist2"]["fin_pose"]["z"])])
            b = np.array([float(data["dist2"]["strt_pose"]["x"]), float(data["dist2"]["strt_pose"]["y"]) -0.26, float(data["dist2"]["strt_pose"]["z"])])
            orientation = self.calculate_rpy(a, b)
            item_pose = Pose(Point(x=(a[0]+b[0])/2, y=(a[1]+b[1])/2, z=(a[2]+b[2])/2),   orientation)
            self.state_msg.model_name = "dist2"
            self.state_msg.pose = item_pose
            self.model_move.publish(self.state_msg)

            a = np.array([float(data["dist3"]["fin_pose"]["x"]), float(data["dist3"]["fin_pose"]["y"]) -0.26, float(data["dist3"]["fin_pose"]["z"])])
            b = np.array([float(data["dist3"]["strt_pose"]["x"]), float(data["dist3"]["strt_pose"]["y"]) -0.26, float(data["dist3"]["strt_pose"]["z"])])
            orientation = self.calculate_rpy(a, b)
            item_pose = Pose(Point(x=(a[0]+b[0])/2, y=(a[1]+b[1])/2, z=(a[2]+b[2])/2),   orientation)
            self.state_msg.model_name = "dist3"
            self.state_msg.pose = item_pose
            self.model_move.publish(self.state_msg)
        else:
            self.state_msg.pose = self.pose
            self.state_msg.model_name = "dist0"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "dist1"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "dist2"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "dist3"
            self.model_move.publish(self.state_msg)
    
    def mam_chkbx_callback(self, msg):
        self.cylinder_chkbx = json.loads(msg.data)["gui"]["cylinder"]
        self.joint_chkbx = json.loads(msg.data)["gui"]["joint_dist"]
        if not self.cylinder_chkbx:
            self.state_msg.pose = self.pose
            self.state_msg.model_name = "cylinder1"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "cylinder2"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "cylinder4"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "cylinder5"
            self.model_move.publish(self.state_msg)

        if not self.joint_chkbx:
            self.state_msg.pose = self.pose
            self.state_msg.model_name = "dist0"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "dist1"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "dist2"
            self.model_move.publish(self.state_msg)
            self.state_msg.model_name = "dist3"
            self.model_move.publish(self.state_msg)


    def calculate_rpy(self, vector1, vector2):
        # print(vector1[2], vector2[2])
        yaw = np.arctan((vector2[1]-vector1[1])/(vector2[0]-vector1[0])) + 1.570796
        roll = np.arctan((vector2[2]-vector1[2])/(vector2[1]-vector1[1])) + 1.570796
        pitch = np.arctan((vector2[2]-vector1[2])/(vector2[0]-vector1[0])) + 1.570796
        if vector1[2] > vector2[2]:
            pitch = np.arctan((vector2[2]-vector1[2])/(vector2[0]-vector1[0])) - 1.570796
        else:
            pitch = np.arctan((vector2[2]-vector1[2])/(vector2[0]-vector1[0])) + 1.570796

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw) # Quaternion(0,0,0,1)
        orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        return orientation
    
    def move_cylinder(self, name, data):
        pose = Pose()
        pose.position.x = float(data["pose"]["x"])
        pose.position.y = float(data["pose"]["y"]) - 0.26
        pose.position.z = float(data["pose"]["z"])
        if name == "cylinder2":
            pose.position.z += 0.171

        pose.orientation.x = float(data["pose"]["or_x"])
        pose.orientation.y = float(data["pose"]["or_y"])
        pose.orientation.z = float(data["pose"]["or_z"])
        pose.orientation.w = float(data["pose"]["or_w"])
        self.state_msg.model_name = name
        self.state_msg.pose = pose
        self.model_move.publish(self.state_msg)

    def move_dist(self, name, data):
        a = np.array([float(data["dist3"]["fin_pose"]["x"]), float(data["dist3"]["fin_pose"]["y"]) -0.26, float(data["dist3"]["fin_pose"]["z"])])
        b = np.array([float(data["dist3"]["strt_pose"]["x"]), float(data["dist3"]["strt_pose"]["y"]) -0.26, float(data["dist3"]["strt_pose"]["z"])])
        orientation = self.calculate_rpy(a, b)
        item_pose = Pose(Point(x=(a[0]+b[0])/2, y=(a[1]+b[1])/2, z=(a[2]+b[2])/2),   orientation)
        self.state_msg.model_name = "dist3"
        self.state_msg.pose = item_pose
        self.model_move.publish(self.state_msg)

if __name__ == '__main__':
    DigitalTwinOdt()
