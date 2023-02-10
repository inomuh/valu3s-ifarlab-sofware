#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""SRVT Image Service Node - SRVT ROKOS'un görüntü kaydı almasını sağlayan düğümdür"""

import rospy
import srvt_moveit.srv as srvt_srv
from class_image_saver import ImageSaver


class ImageServiceClass:
    """Image Service Class"""

    def __init__(self):
        """ImageSaver objesini oluşturuyor"""
        self.left_image_class = ImageSaver("left_rokos")
        self.right_image_class = ImageSaver("right_rokos")

        self.left_rokos_request = str()
        self.right_rokos_request = str()

    def main_func(self):
        """Main Func"""
        rospy.Service(
            "left_rokos_image_service",
            srvt_srv.ImageService,
            self.left_rokos_image_service_func,
        )
        rospy.Service(
            "right_rokos_image_service",
            srvt_srv.ImageService,
            self.right_rokos_image_service_func,
        )

        rospy.spin()

    def left_rokos_image_service_func(self, request):
        """Sol Rokosta görüntü kaydı tutar"""
        try:
            # request olarak image name aliyor
            self.left_rokos_request = request.request
            print("\n\n" + str(self.left_rokos_request))

            color_image = self.left_image_class.color_image_saver_func(
                self.left_rokos_request
            )
            print("\n--> color image = " + str(color_image))

            tof_image = self.left_image_class.tof_image_saver_func(
                self.left_rokos_request
            )
            print("\n--> tof image = " + str(tof_image))

            if color_image is True and tof_image is True:
                response = "succeeded"

            else:
                response = "aborted"

            # Smach'teki outcome'u temsil ediyor.
            return srvt_srv.ImageServiceResponse(response)

        except Exception as err:
            print(err)
            return None

    def right_rokos_image_service_func(self, request):
        """Sağ rokosta görüntü kaydı tutar"""
        try:
            self.right_rokos_request = request.request
            print("\n\n" + str(self.right_rokos_request))

            color_image = self.right_image_class.color_image_saver_func(
                self.right_rokos_request
            )
            print("\n--> color image = " + str(color_image))

            tof_image = self.right_image_class.tof_image_saver_func(
                self.right_rokos_request
            )
            print("\n--> tof image = " + str(tof_image))

            if color_image is True and tof_image is True:
                response = "succeeded"

            else:
                response = "aborted"

            return srvt_srv.ImageServiceResponse(response)

        except Exception as err:
            print(err)
            return None


if __name__ == "__main__":
    rospy.init_node("image_service_node")
    print("Image Service Node Started!")
    image_class = ImageServiceClass()
    image_class.main_func()
