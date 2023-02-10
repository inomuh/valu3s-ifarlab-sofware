#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROKOS sisteminin ana class yapısıdır.
"""
import copy
import math

class RokosClass():
    """
        Rokos Class
    """

    @classmethod
    def calculate_camera_distance_func(cls, task_list, current_degree, rokos_type=True,\
        radian_type=True):
        """
            Kamera transform fonksiyonu

            örn
            Not* = Origin kameranın merceği
            0 derece max robot kolu Y ekseninde 2.84m gidebiliyor.
            90 derecede max 2.72m gidebilir
            180 derecede max 2.60m gidebilir.
            Robot kolunun max limitleri aşılmaması için transform yapılmaktadır.
            (Kamera ve robot kolunun Y ekseni arasında)
        """
        if radian_type:
            current_degree = math.degrees(float(current_degree))

        if rokos_type:  # Right
            max_distance = -0.12

        else:           # Left
            max_distance = 0.12

        if abs(current_degree) >= 360:
            current_degree = abs(current_degree) - 360

        if abs(current_degree) > 180:
            current_degree = 360 - abs(current_degree)

        cam_pose_y_diff = (float(max_distance / 90) * abs(current_degree))

        # Transform sadece Yaw ekseninde gerçekleştirilmektedir.
        if task_list[1] is not None:
            task_list[1] = float(task_list[1] - cam_pose_y_diff)

        return task_list

    @classmethod
    def set_custom_current_task_func(cls, current_task):
        """
            X ve Z ekseninde aynı anda hareket ederek daha sonra Y ekseninde
            hareket etmesi için kullanılır. X,Y,Z gelir ve [X, None, Z],
            [None, Y, None] formatına döner.
        """
        try:
            current_task_list = []
            current_task_type = True

            task_position_x_z = [current_task[0], None, current_task[2]]

            if current_task[1] is None:
                current_task_list.extend(task_position_x_z)
                current_task_type = False

            else:
                task_position_y = [None, current_task[1], None]
                current_task_list.append(task_position_x_z)
                current_task_list.append(task_position_y)

            return current_task_list, current_task_type

        except Exception as err:
            print(err)
            return None

    @classmethod
    def set_new_task_func(cls, current_task, last_position):
        """
            Current task ile last position arasındaki koordinatları karşılaştırır.
            Eğer aynı pozisyon var ise None atar.
        """
        try:
            temp_current_task = copy.deepcopy(current_task)
            temp_last_position = copy.deepcopy(last_position)
            # current_task_list = []

            # for i in range(3):
            #     if temp_last_position[i] == temp_current_task[i]:
            #         temp_current_task[i] = None

            #     current_task_list.append(temp_current_task[i])

            return temp_current_task #current_task_list

        except Exception as err:
            print(err)
            return None


    def set_custom_current_task_new_func(self, current_task, task_control, last_position):
        """
            Task control ilk olarak hangi eksende hareket edeceğini temsil eder.
            Eğer True ise ilk önce X,Z eksenlerine öncelik verir.
            False ise Y eksenine öncelik vermektedir.

        """
        try:
            temp_current_task = copy.deepcopy(current_task)
            temp_last_position = copy.deepcopy(last_position)
            current_task_control = copy.deepcopy(task_control)
            current_task_list = []
            current_task_type = True

            for i in range(3):
                if temp_last_position[i] == temp_current_task[i]:
                    temp_current_task[i] = None

            task_position_x_z = [temp_current_task[0], None, temp_current_task[2]]
            task_position_y = [None, temp_current_task[1], None]

            x_z_control = self.task_control_func(task_position_x_z)
            y_control = self.task_control_func(task_position_y)

            if x_z_control and y_control:
                if current_task_control:
                    current_task_list.append(task_position_x_z)
                    current_task_list.append(task_position_y)

                else:
                    current_task_list.append(task_position_y)
                    current_task_list.append(task_position_x_z)

            elif not x_z_control and not y_control:
                current_task_list.append(False)
                current_task_type = False

            else:
                if not y_control:
                    current_task_list.extend(task_position_x_z)
                    current_task_type = False

                else:
                    current_task_list.extend(task_position_y)
                    current_task_type = False

            return current_task_list, current_task_type

        except Exception as err:
            print(err)
            return None

    @classmethod
    def task_control_func(cls, task_list):
        """
            Listedeki tüm pozisyon değerleri control eder, None ise False döner.
        """
        try:
            counter = 0

            for item in task_list:
                if item is None:
                    counter += 1

            if counter == len(task_list):
                return False
            return True

        except Exception as err:
            print(err)
            return None

    @classmethod
    def new_task_control_func(cls, task_list):
        """
            kaç eksende hareket olduğunu belirleyip, planning time için bu değer kullanılmaktadır.
            Hiç bir eksende hareket etmiyecekse False olarak return etmektedir.
        """
        try:
            counter = 0
            for item in task_list:
                if item is None:
                    counter += 1

            diff_count = (len(task_list) - counter)

            if counter == len(task_list):
                return False, diff_count
            return True, diff_count

        except Exception as err:
            print(err)
            return None

    @classmethod
    def home_position_tolerance_control_func(cls, home_position_list, position_list, tolerance):
        """
            Şuan kullanılmamaktadır.
            Home pozisyona göre mevcut konumu karşılaştırıyor.
        """
        position_x = position_y = position_z = 0
        tolerance = 0.1

        if position_list[0] is not None:
            position_x = abs(home_position_list[0] - position_list[0])

        if position_list[1] is not None:
            position_y = abs(home_position_list[1] - position_list[1])

        if position_list[2] is not None:
            position_z = abs(home_position_list[2] - position_list[2])

        if (position_x < tolerance) and (position_y < tolerance) and (position_z < tolerance):
            return True
        return False

    @classmethod
    def camera_tolerance_control_func(cls, current_robot_positions, position_list, tolerance=0.01):
        """
            Kameranın mevcut konumu ile görevde gelen joint değerleri karşılaştırılmaktadır.
        """
        try:
            cam_1_diff = abs(float(current_robot_positions[4] - position_list[0]))
            cam_2_diff = abs(float(current_robot_positions[5] - position_list[1]))

            if (cam_1_diff < tolerance) and (cam_2_diff < tolerance):
                return True
            return False

        except Exception as err:
            print(err)
            return None
