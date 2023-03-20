#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rokos Smach Lib for right rokos"""
#import roslib
import rospy
import smach
import smach_ros

import rokos_smach_lib_realtime as rsl

def right_rokos_smach_main_func():
    """Smach main function"""
    try:
        rospy.init_node('task_planner_and_pilot_node')
        rokos_type = ""

        # Create the top level SMACH state machine
        sm_top = smach.StateMachine(outcomes=['done'])

        # Open the container
        with sm_top:

            smach.StateMachine.add('Start_State', rsl.SRVTSmach(),
                                transitions={'succeeded':'Move_CON', 'aborted': 'Start_State'})

            # Create the sub SMACH state machine
            sm_move_con = smach.StateMachine(outcomes=['rokos_right_done'])
            sm_move_con.userdata.task = []

            # Open the container
            with sm_move_con:
                smach.StateMachine.add('Rokos_Right_Get_Tasks',
                 rsl.RokosGetTasksState("right_rokos_task_service"),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'Rokos_Right_Get_Tasks',
                                                 'emg_stop': 'Emergency',
                                                 'manuel': 'ManuelState',
                                                 'other': 'rokos_right_done'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task'})

                smach.StateMachine.add('General_Selection',
                 rsl.GeneralSelectionState(),
                                    transitions={'Mobile_Platform_Move': 'Mobile_Platform_Move',
                                                 'Manipulator_Move': 'Manipulator_Move',
                                                 'Manuel_State': 'ManuelState',
                                                 'Rokos_Camera': 'Rokos_Right_Camera',
                                                 'Rokos_Take_Photo': 'Rokos_Right_Take_Photo',
                                                 'Get_Task': 'Rokos_Right_Get_Tasks'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_output':'current_task',
                                                'task_id_output':'task_id'})


                smach.StateMachine.add('Mobile_Platform_Move',
                 rsl.MobilePlatformMoveState(),
                                    transitions={'succeeded':'Manipulator_Move',
                                                 'aborted':'Mobile_Platform_Move',
                                                 'emg_stop': 'Emergency',
                                                 'manuel':'ManuelState',
                                                 'general':'General_Selection'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})

                smach.StateMachine.add('Manipulator_Move',
                 rsl.ManipulatorMoveState(),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'Manipulator_Move',
                                                 'emg_stop': 'Emergency',
                                                 'manuel': 'ManuelState'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})

                smach.StateMachine.add('Rokos_Right_Camera',
                 rsl.RokosCameraState(),
                                    transitions={'succeeded':'Rokos_Right_Take_Photo',
                                                 'aborted':'Rokos_Right_Camera'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})

                smach.StateMachine.add('Rokos_Right_Take_Photo',
                 rsl.RokosTakePhotoState("right_rokos_image_service"),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'Rokos_Right_Take_Photo',
                                                 'emg_stop': 'Emergency'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})

                smach.StateMachine.add('Emergency',
                 rsl.EmergencyState(),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'Emergency'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})

                smach.StateMachine.add('ManuelState',
                 rsl.ManuelState(),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'ManuelState',
                                                 'emg_stop': 'Emergency',
                                                 'task_state': 'Rokos_Right_Get_Tasks'})

            smach.StateMachine.add('Move_CON', sm_move_con,
                                transitions={'rokos_right_done':'done'})


        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROKOS_RIGHT_TASK_SMACH')
        sis.start()
        sm_top.execute()
        rospy.spin()
        sis.stop()

    except Exception as err:
        print(err)

if __name__ == '__main__':
    right_rokos_smach_main_func()
