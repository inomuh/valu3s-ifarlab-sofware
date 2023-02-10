#!/usr/bin/env python3
# coding=utf-8

"""
Service provider
Publishes acceleration depending cmd_vel command

@Author: Furkan Edizkan
@Task: #IFARLAB
"""


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from mobile_manipulator_action.srv import AccelerationService, AccelerationServiceResponse, AccelerationServiceRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

ODOM_POSITION_X = None
cancel_data = 0

def ServiceCallback(req):
    global ODOM_POSITION_X, cancel_data
    rate_param = 20.0
    rospy.loginfo("acceleration_service::Received request == %s, %s", str(req.pos_x), str(req.vel_config))
    resp = AccelerationServiceResponse()
    resp.response = 'Failed to get acceleration'
    rospy.Subscriber("/mp_action/cancel", Int8, cancel_callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    cmd_msg = Twist()

    pos_x, vel_config = req.pos_x, abs(req.vel_config)
    
    if vel_config > 0.25:
        vel_config = 0.25
    elif vel_config < 0.0:
        vel_config = 0.0
        
    if pos_x > 2.08:
        pos_x = 2.08
    elif pos_x < 0.0:
        pos_x = 0.0
    
    try:
        vel = 0
        pose_status = False
        
        while not rospy.is_shutdown():
            rospy.Rate(rate_param).sleep()
            diff = pos_x - ODOM_POSITION_X
            acc_param = np.sign(diff) * (vel_config/rate_param)
            if cancel_data:
                cmd_msg.linear.x = 0            
                pub.publish(cmd_msg)
                resp.response = 'Aborted'
                cancel_data = 0
                break
            else:
                if abs(diff) > abs(vel_config)/2:
                    vel += acc_param
                    move_forward = True
                    if abs(vel) > abs(vel_config):
                        vel = np.sign(diff) * vel_config                                 
                else:
                    if not move_forward:
                        vel = np.sign(diff) *  0.08
                    else:
                        vel = diff

                if abs(diff) < 0.005:
                    vel = 0
                    pose_status = True
            
            cmd_msg.linear.x = vel
            
            pub.publish(cmd_msg)
            
            if pose_status:
                resp.response = 'Succeeded'
                rospy.loginfo("Succeeded")
                break

    except Exception as err:
        rospy.loginfo("acceleration_service::Error == %f", err)
        resp.response = 'Aborted'
    
    return resp


def OdomCallback(msg):
    """Odometry callback"""
    global ODOM_POSITION_X
    ODOM_POSITION_X = msg.pose.pose.position.x
    
def cancel_callback(msg):
    global cancel_data
    cancel_data = msg.data
        
if __name__ == '__main__':
    rospy.init_node('acceleration_service')
    rospy.Subscriber("/odom", Odometry, OdomCallback)
    service = rospy.Service('acceleration_service', AccelerationService, ServiceCallback)
    rospy.loginfo("acceleration_service service ready")
    rospy.spin()