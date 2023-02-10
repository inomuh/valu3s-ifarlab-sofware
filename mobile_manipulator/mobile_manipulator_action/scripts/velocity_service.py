#!/usr/bin/env python3
# coding=utf-8

"""
Service provider
Publishes given cmd_vel command

@Author: Furkan Edizkan
@Task: #IFARLAB
"""

import rospy
from geometry_msgs.msg import Twist
from mobile_manipulator_action.srv import VelocityService, VelocityServiceResponse, VelocityServiceRequest

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
cmd_msg = Twist()

def ServiceCallback(req):
    vel = req.cmd_msg.linear.x
    if vel > 0.25:
        vel = 0.25
    cmd_msg.linear.x = vel
    cmd_msg = req.cmd_msg
    rospy.loginfo("velocity_service::Received request == \n%s", str(cmd_msg))
    pub.publish(cmd_msg)
    
    resp = VelocityServiceResponse()
    return resp

        
if __name__ == '__main__':
    rospy.init_node('velocity_service')
    service = rospy.Service('velocity_service', VelocityService, ServiceCallback)
    rospy.loginfo("velocity_service service ready")
    rospy.spin()