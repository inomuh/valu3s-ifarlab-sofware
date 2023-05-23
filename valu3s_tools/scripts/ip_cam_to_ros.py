#!/usr/bin/env python3

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image

class IpCamera2Ros():
    def __init__(self):
        rospy.init_node("ip_camera_to_ros")
        self.url = "rtsp://admin:ifarlab_2023.@192.168.3.5:554"
        self.fps = 0
        self.pub = rospy.Publisher("env_cam", Image, queue_size=30)
        self.rate = rospy.Rate(30)
        self.image = Image()
        self.main()

    def main(self):
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        if cap.isOpened():
            while not rospy.is_shutdown():
                ret, frame = cap.read()
                if ret:
                    height, width, channels = frame.shape
                    self.image.height = height
                    self.image.width = width
                    self.image.encoding = "rgb8"
                    self.image.header.stamp = rospy.Time.now()
                    self.image.header.frame_id = "env_cam"
                    cv_rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    self.image.data = bytes(cv_rgb_image)
                    self.image.step = width * 1 * channels
                    self.pub.publish(self.image)
                    # self.image.data = []
                else:
                    break
                self.rate.sleep()

if __name__ == "__main__":
     IpCamera2Ros()