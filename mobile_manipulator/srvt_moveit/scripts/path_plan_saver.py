"""Path plan saver Class"""
import os
import json

class PathPlanSaver():
    """Path Plan saver class"""
    def __init__(self, current_plan):
        self.current_plan = current_plan
        self.task_counter = 1
        self.tasks = []
    def record_path(self):
        """Path record function"""
        self.tasks.append(str(self.current_plan))
        with open(str(self.get_current_workspace())+\
            '/plan_data/rokos_path_plan.json', 'a', encoding='utf8') as plan_file:
            name = 'task_'+str(self.task_counter)
            self.task_counter+=1
            json.dump({name: self.tasks}, plan_file, indent=1)
            plan_file.write('\n')

    @classmethod
    def get_current_workspace(cls):
        """
        get_current_workspace
        """
        file_full_path = os.path.dirname(os.path.realpath(__file__))

        #srvt_ws_location = file_full_path.split('/', 10)
        #srvt_ws_location = '/'+srvt_ws_location[1]+'/'+ srvt_ws_location[2]+'/'+\
        #     srvt_ws_location[3]+'/'+ srvt_ws_location[4]+'/'+srvt_ws_location[5]
        return file_full_path
