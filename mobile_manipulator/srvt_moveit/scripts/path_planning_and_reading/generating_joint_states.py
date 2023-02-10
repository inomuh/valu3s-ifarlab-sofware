"""
Bu script, .yaml uzantili, robotun "joint_state" yayinindan alinan yaml dosyasini
isleyip konum, hiz ve effort bilgilerinin ayriştirilarak bir .csv dosyasina 
kaydedilmesini saglar.

Created by: Alim Kerem Erdogmus - 12.04.22
"""
import yaml
from yaml.loader import SafeLoader
import csv
from tf.transformations import euler_from_quaternion
from numpy import rad2deg

target_point_positions = []
target_point_velocities = []
target_point_efforts = []
fname = "None"

def read_plan_file(file_name, file_format):
    """
    Main function
    """
    global fname # her fonksiyondan ulaşılabilir olması için file_name değişkeni global tanımlandı.
    fname = file_name

    # Open the file and load the file
    with open(file_name+file_format) as f:
        data = list(yaml.load_all(f, Loader=SafeLoader))

    joint_names = get_joint_names_from_plan(data)

    for i in range(len(data)-1):
        """
        "data" listesi ile rotayi oluşturan noktalar arasi izlenmesi gereken
        yol bilgisi işlenmiştir. Her eleman "selecting_datas_from_plan" fonksiyonu ile
        ayiklanir (positions, velocities ve accelerations bilgileri alinir). Ayiklanan
        bilgiler "target_position_<parametre_adi>" listelerine işlenmek için "get_target
        position_datas" fonksiyonuna gönderilir.
        """
        points, velocities, efforts = selecting_datas_from_plan(data, i)
        set_target_position_datas(points, velocities, efforts)

    # Elde edilen tüm rota koordinat bilgileri önce terminalden gösterilir, sonra csv
    # dosyasına kaydedilmek üzere "save_points_file" fonksiyonuna gönderilir.
    #show_results(target_point_positions, target_point_velocities, target_point_acc, tl_jt_joint_names)
    save_points_file(target_point_positions, target_point_velocities, target_point_efforts, joint_names)

def selecting_datas_from_plan(data, i):

    position_info = data[i]
    position_points = position_info["position"]
    position_velocity = position_info["velocity"]
    position_effort = position_info["effort"]

    return position_points, position_velocity, position_effort

def get_joint_names_from_plan(data):

    position_info = data[0] # dict
    joint_names = position_info["name"]

    return joint_names

def set_target_position_datas(pts, vel, eff):

    target_point_positions.append(pts)
    target_point_velocities.append(vel)
    target_point_efforts.append(eff)

def save_points_file(tpp, tpv, tpe, joint_names):
    """
    Bu fonksiyonda, ayiklanan plan dosyasindan elde edilen bilgiler .csv dosyasina kaydedilir.
    Position parametresinde ise "quaternion_to_euler" işlemi uygulanarak Orientation X,Y,Z,W bilgileri
    Raw, Pitch, Yaw'a dönüştürülerek kayit yapilir.
    """
    arr = [0,0,0,0,0,0,0]
    points_file_name = fname+"_points"
    with open(points_file_name+".csv", mode='w') as points_file_name:
        points_file = csv.writer(points_file_name, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        points_file.writerow(["Points"]+joint_names)
        for position in tpp: 
            arr[1:] = position
            arr[0]+=1
            points_file.writerow(arr)

        arr[0]=0
        arr.append(0)

        points_file.writerow(" ")
        points_file.writerow(["Velocities"]+joint_names)

        for velocity in tpv:
            arr[1:] = velocity
            arr[0]+=1
            points_file.writerow(arr)

        arr[0]=0

        points_file.writerow(" ")
        points_file.writerow(["Efforts"]+joint_names)
        for acc in tpe:
            arr[1:] = acc
            arr[0]+=1
            points_file.writerow(arr)

def transform_quaternion(quat):
    """
    Bu fonksiyonda Orientation X, Y, Z, W parametreleri Raw, Pitch, Yaw değerlerine dönüstürülür.
    (derece olarak).
    """
    rpy = rad2deg(euler_from_quaternion(quat))
    #print("The euler representation is ",quat[0],quat[1],quat[2])
    return rpy

read_plan_file("rightrokos_joint_states_1304",".yaml")
