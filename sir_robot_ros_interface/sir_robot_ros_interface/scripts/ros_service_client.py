#!/usr/bin/env python3
# coding=utf-8

import rospy
from sir_robot_ros_interface.srv import ManipulatorPose_ino_2, ManipulatorPose_ino_2Request, ManipulatorPose_ino_2Response
from sir_robot_ros_interface.msg import Pose, Poses


def call_service(srv_name, srv_type, srv_request=None):
    """Call input service function"""
    client = rospy.ServiceProxy(srv_name, srv_type)
    rospy.wait_for_service(srv_name, timeout=5)
    
    if srv_request == None:
        response = client()
    else:
        response = client(srv_request)
        
    #response = response.<resposne_name> # return value of the service call
    
    return response


if __name__ == '__main__':
    req = ManipulatorPose_ino_2Request()
    
    req.path = "/home/ifarlab/catkin_ws/src/sir_robot_ros_interface/sir_robot_ros_interface/csv/35269.csv"
    print(req)
    
    resp = call_service(srv_name="/manipulator_service_test", srv_type=ManipulatorPose_ino_2, srv_request=req.path)
    print(resp.status)