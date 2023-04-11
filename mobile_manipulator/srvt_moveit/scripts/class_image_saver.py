#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Class image server
"""
import os
import sys
from datetime import datetime
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class ImageSaver():
    """
    Camera rgb image saver
    """
    def __init__(self, g_name=""):
        self.group_name = str(g_name)
        self.current_color_image = None
        self.current_tof_image = None

        self.current_workspace = self.get_current_workspace()
        self.dir_name = str(self.current_workspace) + 'srvt_moveit/image_file/' +\
             str(self.group_name)

        self.bridge = CvBridge()
        # import kısmında "as bridge" şeklinde alındığında hata alınıyor.
        self.color_cam_sub = rospy.Subscriber(('/sick_visionary_t_mini/intensity'),\
         Image, self.__color_cam_callback)
        self.tof_cam_sub = rospy.Subscriber(('/sick_visionary_t_mini/depth'),\
         Image, self.__tof_cam_callback)


    def __color_cam_callback(self, msg):
        self.current_color_image = msg


    def __tof_cam_callback(self, msg):
        self.current_tof_image = msg


    def color_image_saver_func(self, image_name):
        """RGB resimlerin kaydedildiği fonk"""
        try:
            if self.current_color_image is not None:
                #time.sleep(1)
                #self.rokos_color_image_name = str(image_name)
                temp_image = self.current_color_image
                self.__display(temp_image, "color_image", str(image_name))
                #self.color_cam_control = True

                return True
            return False

        except Exception as err:
            print(err)
            return None


    def tof_image_saver_func(self, image_name):
        """Tof resimlerin kaydedildiği fonk"""
        try:
            if self.current_tof_image is not None:
                #time.sleep(1)
                #self.rokos_tof_image_name = str(image_name)
                temp_image = self.current_tof_image
                self.__display(temp_image, "tof_image", str(image_name))
                #self.tof_cam_control = True

                return True
            return False

        except Exception as err:
            print(err)
            return None

    def __display(self, img_msg, img_type, img_name):
        try:
            current_time = self.datenow_func()

            if img_type == "tof_image":
                img = self.bridge.imgmsg_to_cv2(img_msg, "32FC1")
                new_dir = self.dir_name + "/tof_image"
                img_file_format = ".pgm"

                #normalization

                image_name = str(img_name + "_" + self.group_name + "_" + img_type +\
                     "_" + current_time + str(img_file_format))
                # The depth image is a single-channel float32 image
                depth_image = self.bridge.imgmsg_to_cv2(img_msg, "32FC1")


                # Convert the depth image to a numpy array since most cv2 functions
                # require numpy arrays

                depth_array = np.array(depth_image, dtype=np.float32)

                # Normalize the depth image to fall between 0 (black) an 1 (white)
                cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

                # Process the depth image
                depth_display_image = depth_array*255
                # saving normalized tof image
                cv2.imwrite(os.path.join(new_dir, image_name), depth_display_image)

            else:
                img = self.bridge.imgmsg_to_cv2(img_msg, "16UC1")
                new_dir = self.dir_name + "/color_image"
                img_file_format = ".jpg"

                image_name = str(img_name + "_" + self.group_name + "_" + img_type +\
                     "_" + current_time + str(img_file_format))
                # saving image
                cv2.imwrite(os.path.join(new_dir, image_name), img)

        except Exception as err:
            print(err)

    @classmethod
    def datenow_func(cls):
        """Datenow func"""
        now = datetime.now()
        dt_string = now.strftime("%Y_%m_%d_-_%H_%M_%S")

        return str(dt_string)

    @classmethod
    def get_current_workspace(cls):
        """
            Get Current Workspace Function
        """
        file_full_path = os.path.dirname(os.path.realpath(__file__))
        directory_name = sys.argv[0].split('/')[-3]
        workspace_name = file_full_path.split(str(directory_name))[0]

        return workspace_name
