#!/usr/bin/env python3

import json
import rospy
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from parameter_configuration.cfg import ui_parametersConfig
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class ParameterConfig:
    def __init__(self):
        rospy.init_node("ui_parameters")
        self.parameters = None
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        Server(ui_parametersConfig, self.parameters_callback)
        self.param_publisher = rospy.Publisher("ui_parameters_topic", String, queue_size=10)
        rospy.Subscriber("ui_set_parameters", String, self.parameter_callback)
        self.param_client = Client("ui_parameters", timeout=15, config_callback = self.config_callback)        
        self.main()

    def main(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                if self.parameters:
                    self.param_publisher.publish(str(self.parameters))
            except:
                pass
            rate.sleep()

    def parameters_callback(self, config, level):
        rospy.loginfo(config)
        self.parameters = config
        self.parameters = "{\"topic_rate\":" + str(config["topic_rate"])+ ",\"robot_velocity\":" + \
            str(config["robot_velocity"]) + ",\"sensor_origin_set\":"+ str(config["sensor_origin_set"]) + "}"
        return config

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_vx = msg.twist.twist.linear.x

    def parameter_callback(self, msg):
        data = json.loads(msg.data)
        self.param_client .update_configuration(
            {"topic_rate": data["topic_rate"], "robot_velocity": data["robot_velocity"], "sensor_origin_set": data["sensor_origin_set"]})

    def config_callback(self, config):
        rospy.loginfo("callback {}",config)

if __name__ == '__main__':
    ParameterConfig()