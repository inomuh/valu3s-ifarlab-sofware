#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    left_rokos_task.yaml ve right_rokos_task.yaml dosyalarinda görevlerin bilgilerini çeker.
    Daha sonra rokos_task.yaml dosyasinda belirtilen görev ID' lerine göre filtreleme
    işlemi uygular. Smach' ten Task Service'e istek gelir ve filtrelenmiş görevleri
    Smach' e gönderir.
"""
import rospy
import srvt_moveit.srv as srvt_srv
#from task_log_node import TaskLogClass

class TaskServiceClass():
    """
        Taskservice class
    """
    def __init__(self):
        # left_rokos_task.yaml ve right_rokos_task.yaml dosyalarında görevlerin bilgilerini okur.
        print("Right_Rokos")
        self.read_right_rokos = rospy.get_param('~Right_Rokos')

        # print("Left_Rokos")
        # self.read_left_rokos = rospy.get_param('~Left_Rokos')

        # Gelen istekleri tutar.
        # self.left_rokos_request = str()
        self.right_rokos_request = str()

        # Kaç kere görev isteği geldiğini tutar.
        self.right_task_counter = 0
        self.left_task_counter = 0

        # Parçalı görevler için True olarak atanır.
        self.right_multi_task_control = False
        # self.left_multi_task_control = False

        # İstek geldikten sonra yanıt olarak bu listedeki görevler gönderilir.
        self.right_rokos_task_list = []
        # self.left_rokos_task_list = []

        # Parçalı görevlerde bu yapı kullanılır.
        self.right_rokos_multi_task_list = []
        # self.left_rokos_multi_task_list = []


        # Parçalı görev için aşağıdaki fonksiyon kullanılır.
        #self.multi_task_list_func()

        # Tek parçada görevi göndermek için aşağıdaki fonksiyon kullanılır.
        self.task_list_func()


    def task_list_func(self):
        """task list func"""
        try:
            right_filter_list = rospy.get_param('~Rokos_Task/Right_Rokos')
            # left_filter_list = rospy.get_param('~Rokos_Task/Left_Rokos')
            #print("\n\nRead Rokos Right Task = {}\n\n".format(right_filter_list))
            #print("\n\nRead Rokos Left Task = {}\n\n".format(left_filter_list))


            # rokos_task.yaml dosyasında belirtilen görev ID' leri yerine None yazılmış
            # ise, tüm görevler gönderilir.

            if right_filter_list == "None":
                right_filter_list = None

            # if left_filter_list == "None":
            #     left_filter_list = None

        except Exception:
            right_filter_list = None
            left_filter_list = None

        # Görev listeleri filtrelere göre oluşturulur.
        self.right_rokos_task_list = self.filter_read_task(self.read_right_rokos, right_filter_list)
        # self.left_rokos_task_list = self.filter_read_task(self.read_left_rokos, left_filter_list)

        print(f"\nRokos Right Task = {self.right_rokos_task_list}\n")
        # print(f"\nRokos Left  Task = {self.left_rokos_task_list}\n")


    def readRosParamTaskID():
        """Read ROS parameters"""
        try:
            data = rospy.get_param("/Right_Rokos")
            taskIdList = []
            
            for item in data:
                taskIdList.append(item["Task"]["Task_ID"])
                
            print(taskIdList)
            return data
        
        except Exception as err1:
            print("Error::ROSParam::{}".format(err1))


    def multi_task_list_func(self):
        """2 Parca"""
        
        data = self.readRosParamTaskID()
        right_filter_list_1 = list(data)
        
        #right_filter_list_1 = list([35269,35270,35271,35272,35273])

        right_filter_list_2 = list([35351, 35352, 35353, 35354, 35355, 35356, 35357, 35358,
        35359, 35360, 35361, 35362, 35363, 35364, 35365, 35366, 35367, 35368, 35369, 35370,
        35371, 35372, 35373, 35374, 35375, 35376, 35377, 35378, 35379, 35380, 35381, 35382,
        35385, 35386, 35387, 35388, 35389, 35390, 35391, 35392, 35393, 35394, 35395, 35396,
        35397, 35398, 35399, 35400, 35401, 35402, 35403, 35404, 35405, 35406, 35407, 35408,
        35409, 35410, 35411, 35412, 35413, 35414, 35415, 35416, 35417, 35418, 35419, 35420,
        35421, 35422, 35423, 35424, 35425, 35426, 35427, 35428, 35429, 35430, 35431, 35432,
        35433, 35434, 35435, 35436])

        # left_filter_list_1 = list([35694, 35695, 35696, 35697, 35698, 35699, 35700, 35701,
        # 35702, 35703, 35704, 35705, 35706, 35707, 35708, 35709, 35710, 35711, 35712, 35713,
        # 35714, 35715, 35716, 35717, 35718, 35719, 35720, 35721, 35722, 35723, 35724, 35725,
        # 35726, 35727, 35728, 35729, 35730, 35731, 35732, 35733, 35734, 35735, 35736, 35737,
        # 35738, 35739, 35740, 35741, 35742, 35743, 35744, 35745, 35746, 35747, 35748, 35749,
        # 35751, 35752, 35753, 35756, 35757, 35758, 35759, 35760, 35761, 35762, 35763, 35764,
        # 35765, 35766, 35767, 35768, 35769, 35770, 35771, 35772, 35773, 35774, 35775, 35776,
        # 35777, 35778, 35779, 35780, 35781, 35782, 35783])

        # left_filter_list_2 = list([35784, 35785, 35786, 35787, 35788, 35789, 35790, 35791,
        # 35792, 35793, 35794, 35795, 35796, 35797, 35798, 35799, 35800, 35801, 35802, 35803,
        # 35804, 35805, 35806, 35807, 35808, 35809, 35810, 35811, 35812, 35813, 35814, 35815,
        # 35816, 35817, 35818, 35819, 35820, 35821, 35822, 35823, 35824, 35825, 35826, 35827,
        # 35828, 35829, 35830, 35831, 35832, 35833, 35834, 35835, 35836, 35837, 35838, 35839,
        # 35840, 35841, 35842, 35843, 35844, 35845, 35846, 35847, 35848, 35849, 35850, 35851,
        # 35852, 35853, 35854, 35855, 35858, 35859, 35860, 35861, 35862, 35863, 35864, 35865,
        # 35866, 35867, 35868, 35869, 35870, 35871, 35872, 35873, 35874, 35875, 35876, 35877,
        # 35878, 35879, 35880, 35881, 35882])

        # Görevler parçalı olarak oluşturulur.
        temp_1 = self.filter_read_task(self.read_right_rokos, right_filter_list_1)
        # temp_2 = self.filter_read_task(self.read_right_rokos, right_filter_list_2)
        self.right_multi_task_control = True
        self.right_rokos_multi_task_list.append(temp_1)
        # self.right_rokos_multi_task_list.append(temp_2)


        # temp_1 = self.filter_read_task(self.read_left_rokos, left_filter_list_1)
        # temp_2 = self.filter_read_task(self.read_left_rokos, left_filter_list_2)
        # self.left_multi_task_control = True
        # self.left_rokos_multi_task_list.append(temp_1)
        # self.left_rokos_multi_task_list.append(temp_2)


    def main_func(self):
        """ Task Serviceler oluşturulur. """
        # left_rokos_task_service = rospy.Service('left_rokos_task_service', srvt_srv.TaskService,
        #  self.left_rokos_task_service_func)
        right_rokos_task_service = rospy.Service('right_rokos_task_service', srvt_srv.TaskService,
         self.right_rokos_task_service_func)

        rospy.spin()

    # def left_rokos_task_service_func(self, request):
    #     """Sol rokos için görev servisi"""
    #     try:
    #         self.left_rokos_request = request.request
    #         print(self.left_rokos_request)

    #         if self.left_multi_task_control:
    #             task_index = self.left_task_counter % len(self.left_rokos_multi_task_list)
    #             response = str(self.left_rokos_multi_task_list[task_index])

    #         else:
    #             response = str(self.left_rokos_task_list)

    #         self.left_task_counter += 1

    #         return srvt_srv.TaskServiceResponse(response)

    #     except Exception as err:
    #         print(err)
    #         return None


    def right_rokos_task_service_func(self, request):
        """ Sag rokos için görev servisi"""
        try:
            self.right_rokos_request = request.request
            print(self.right_rokos_request)

            # Multi task için counterın görev listesine göre modunu alır.
            # Örneğin 2 parça görevde, indis 0 ve indis 1 i sürekli olarak
            # göndermesi için.
            if self.right_multi_task_control:
                task_index = self.right_task_counter % len(self.right_rokos_multi_task_list)
                response = str(self.right_rokos_multi_task_list[task_index])

            else:
                response = str(self.right_rokos_task_list)

            self.right_task_counter += 1

            return srvt_srv.TaskServiceResponse(response)

        except Exception as err:
            print(err)
            return None

    @classmethod
    def filter_read_task(cls, read_task, filter_list):
        """
            Tüm görevlenden(read_task), yapilmasi istenen görev IDlerine(filter_list) göre
            filtreleme işlemi burada gerçekleşir.
        """
        try:
            mission_list = []
            """
            - Task:
                Task_ID: 35694

                Vehicle_CODE: '13M38_1'

                Mode: 1

                Tag: 'ON'

                Position:
                    X: 6.079
                    Y: 2.840
                    Z: 1.603
                    C_AX: 10.000
                    C_AZ: 170.000

                Queue:
                    X: 1
                    Y: 3
                    Z: 2
                    C_AX: 4
                    C_AZ: 5

            Not* = left_rokos_task.yaml ve right_rokos_task.yaml dosyalarinda bulunan
            görevlerden filter_list'te yer alan IDlere göre görevlerin bilgilerini okur.
            Okunan bilgileri uygun şablona getirerek istemciye iletir.
            Şablon = [[Position], [Queue], [Info]]
            """
            for _, item in enumerate(read_task):
                position = item["Task"]["Position"]
                # queue = item["Task"]["Queue"]
                mode = item["Task"]["Mode"]
                tag = item["Task"]["Tag"]
                task_id = item["Task"]["Task_ID"]
                vehicle_code = item["Task"]["Vehicle_CODE"]

                if filter_list is not None:
                    if task_id not in filter_list:
                        continue
                task_list = list([[position["X"], position["Y"], position["Z"], position["roll"],
                 position["pitch"], position["yaw"]], [1, 2, 3, 4, 5], [task_id, str(vehicle_code), mode, str(tag)]])

                # task_list = list([[position["X"], position["Y"], position["Z"], position["roll"],
                #  position["pitch"], position["yaw"]], [queue["X"], queue["Y"], queue["Z"], queue["C_AX"],
                #   queue["C_AZ"]], [task_id, str(vehicle_code), mode, str(tag)]])

                mission_list.append(task_list)

            return mission_list

        except Exception as err:
            print(err)
            return None


if __name__ == '__main__':
    rospy.init_node('task_ui_node')
    task_class = TaskServiceClass()
    task_class.main_func()
