"""
Bu script, .yaml uzantili, moveitten alinan trajectory plan dosyasini isleyip
konum, hiz ve ivme bilgilerinin ayriştirilarak bir .csv dosyasina 
kaydedilmesini saglar.

Created by: Alim Kerem Erdogmus - 05.04.22
"""
import yaml
from yaml.loader import SafeLoader
import csv
from tf.transformations import euler_from_quaternion
from numpy import rad2deg

target_point_positions = []
target_point_velocities = []
target_point_acc = []
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

    tl_jt_joint_names = get_joint_names_from_plan(data)
    print(tl_jt_joint_names) 
    # tl_jt_points: trajectory list's joint trajectories' points -> Toplanan planned_path.yaml dosyasında
    # trajectory bölümünün altındaki joint_trajectory parametrelerinin points değerleri bu değişkende
    # toplanır.

    for i in range(len(data)-1):
        """
        "data" listesi ile rotayi oluşturan noktalar arasi izlenmesi gereken
        yol bilgisi işlenmiştir. Her eleman "selecting_datas_from_plan" fonksiyonu ile
        ayiklanir (positions, velocities ve accelerations bilgileri alinir). Ayiklanan
        bilgiler "target_position_<parametre_adi>" listelerine işlenmek için "get_target
        position_datas" fonksiyonuna gönderilir.
        """
        tl_jt_points = selecting_datas_from_plan(data, i)
        get_target_position_datas(tl_jt_points)

    # Elde edilen tüm rota koordinat bilgileri önce terminalden gösterilir, sonra csv
    # dosyasına kaydedilmek üzere "save_points_file" fonksiyonuna gönderilir.
    show_results(target_point_positions, target_point_velocities, target_point_acc, tl_jt_joint_names)
    save_points_file(target_point_positions, target_point_velocities, target_point_acc, tl_jt_joint_names)

def selecting_datas_from_plan(data, i):
    """
    Bu fonksiyonda planned_path.yaml (ya da herhangi bir plan dosyası) dosyasinin ayiklanmasi
    saglanir. Önce verilerdeki her iki nokta arasi rota bilgisini iceren bilgi "position_info"ya
    alinir. Her position_info içerisinde rotayi olusturan koordinat, hiz ve ivme bilgisini tasir.
    "trajectory_list" degiskeni ile position_info icerisindeli "trajectory" parametresi cekilir.
    Bu liste icerisinde iki dict eleman barindirir. İsimize yarayacak bilgiler bu listenin ilk
    elemanindadir. Bu eleman "tl" degiskenine atilir. "tl" degiskeni icerisindeki "joint_trajectory"
    kütüphanesi "tl_joint_trajectory" icerisine atilir. Bu kütüphane icerisindeki "points" degiskeni
    bize koordinat, hiz ve ivme bilgisini tasiyan degiskendir. Bu bilgiler de "tl_jt_points" icerisine
    cekilerek veri ayiklama isi tamamlanir.
    """
    position_info = data[i] # dict
    trajectory_list = position_info["trajectory"] # list
    tl = trajectory_list[0] # dictionary
    tl_joint_trajectory = tl["joint_trajectory"] # dictionary
    tl_jt_points = tl_joint_trajectory["points"] # list

    return tl_jt_points

def get_joint_names_from_plan(data):
    """
    Bu fonksiyon "selecting_datas_from_plan" fonksiyonundaki yöntemle, plan dosyasindaki
    joint isimlerini (joint_names) bir seferlik alip işlemek üzere gönderilmesini
    saglar.
    """
    position_info = data[0] # dict
    trajectory_list = position_info["trajectory"] # list
    tl = trajectory_list[0] # dictionary
    tl_joint_trajectory = tl["joint_trajectory"] # dictionary
    tl_jt_joint_names = tl_joint_trajectory["joint_names"]

    return tl_jt_joint_names

def get_target_position_datas(tl_jt_points):
    """
    Koordinat verilerinin listeler halinde "positions", "velocities" ve 
    "accelerations" olarak düzenlendiği fonksiyondur.
    """
    for i in range(len(tl_jt_points)):
        target_point = tl_jt_points[i]
        target_point_positions.append(target_point["positions"])
        target_point_velocities.append(target_point["velocities"])
        target_point_acc.append(target_point["accelerations"])

def show_results(tpp, tpv, tpa, joint_names):
    """
    Koordinat bilgilerini terminalde gösteren fonksiyondur.
    """
    print("Joint Names: ", joint_names)
    print("Positions: ")
    for position in tpp:
        print(position)

    print("---")

    print("Velocities: ")
    for velocity in tpv:
        print(velocity)

    print("---")

    print("Accelerations: ")
    for acc in tpa:
        print(acc)

    print("---")

def save_points_file(tpp, tpv, tpa, joint_names):
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
        points_file.writerow(["Accelerations"]+joint_names)
        for acc in tpa:
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

read_plan_file("leftrokos_path_plan_1204",".yaml")
