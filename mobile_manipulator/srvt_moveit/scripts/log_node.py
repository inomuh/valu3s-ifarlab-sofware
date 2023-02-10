#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" SRVT LogClass"""

import os
import sys
import csv
from datetime import datetime
import yaml


class LogClass():
    """SRVT LogClass"""
    def __init__(self, file_name, current_state):
        self.file_name = file_name
        self.current_state = current_state
        self.current_time = self.datenow_func()
        self.current_workspace = self.get_current_workspace()
        self.new_file_name = str(self.current_workspace) + 'srvt_moveit/log_file/' +\
             str(self.current_time + "_" + self.file_name)

        header_list = []
        header_list.append(list(current_state.joint_state.name))
        self.csv_write_func(header_list)
        # linear_x_actuator_joint	linear_z_actuator_joint	linear_y1_actuator_joint
        # linear_y2_actuator_joint	cam1_actuator_joint	cam2_actuator_joint
        # header olarak kaydedilir.
        # [linear_x_actuator_joint	linear_z_actuator_joint	linear_y1_actuator_joint
        # linear_y2_actuator_joint] metre cinsinde
        # [cam1_actuator_joint	cam2_actuator_joint] radian cinsinde değerleri ifade
        # eder.

    def open_file(self, file_name):
        self.current_workspace = self.get_current_workspace()
        self.task_plan_file_name = str(self.current_workspace) + 'srvt_moveit/moveit_task_log/' + file_name
        header_list = []
        header_list.append(list(self.current_state.joint_state.name))
        # self.csv_write_func(header_list)
        with open(self.task_plan_file_name + '.csv','w+', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(header_list)
        csv_file.close()

    def task_plan_save_func(self, raw_data):
        """Main func"""
        write_data = self.split_func(raw_data)
        # self.csv_add_func(write_data)
        with open(self.task_plan_file_name + '.csv','a+', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(write_data)
        csv_file.close()

    def main_func(self, raw_data):
        """Main func"""
        write_data = self.split_func(raw_data)
        self.csv_add_func(write_data)

    def csv_write_func(self, write_data_list):
        """CSV Write func"""
        with open(self.new_file_name + '.csv','w+', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(write_data_list)
        csv_file.close()


    def csv_add_func(self, write_data_list):
        """CSV add func"""
        with open(self.new_file_name + '.csv','a+', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(write_data_list)
        csv_file.close()


    def yaml_write_func(self, write_data_list):
        """Yaml write func"""
        self.current_time = self.datenow_func()
        new_file_name = str(self.current_time + "_" + self.file_name)
        with open(str(self.current_workspace) + 'srvt_moveit/log_file/'+\
             str(new_file_name) + '.yaml', 'w', encoding='utf-8') as outfile:
            yaml.dump(write_data_list, outfile, default_flow_style=False)

    @classmethod
    def split_func(cls, raw_data):
        """Split func"""
        log_list = []

        for item in raw_data.joint_trajectory.points:
            print(raw_data)
            log_list.append(list(item.positions))

        return log_list


    @classmethod
    def datenow_func(cls):
        """Current date function for"""
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
