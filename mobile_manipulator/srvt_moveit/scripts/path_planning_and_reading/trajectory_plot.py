from cv2 import line
from matplotlib import animation
import numpy as np
import matplotlib.pyplot as plt
import csv

from animate_plot import AnimationPlot as ap

xline = []
yline = []
zline = []

def get_coordinate_datas():

    """Left ROKOS"""
    with open('leftrokos_path_plan_1204_points.csv') as csv_file:
        csv_reader = list(csv.reader(csv_file, delimiter=','))
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                line_count+=1
            else:            
                if len(row) != 1:
                    if row[1] is not str:
                        xline.append(row[1])
                    if row[3] is not str:
                        yline.append(str(float(row[3])+float(row[4])))
                    if row[2] is not str:
                        zline.append(row[2])    
                    line_count+=1
                else:
                    print(f'Processed {line_count-1} positions for Left Rokos.')
                    left_trajectory_plot(xline, yline, zline, line_count)
                    break

    """Right ROKOS"""
    with open('rightrokos_path_plan_1304_points.csv') as csv_file:
        csv_reader = list(csv.reader(csv_file, delimiter=','))
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                line_count+=1
            else:            
                if len(row) != 1:
                    if row[1] is not str:
                        xline.append(row[1])
                    if row[3] is not str:
                        yline.append(str(float(row[3])+float(row[4])))
                    if row[2] is not str:
                        zline.append(row[2])    
                    line_count+=1
                else:
                    print(f'Processed {line_count-1} positions for Right Rokos.')
                    right_trajectory_plot(xline, yline, zline, line_count)
                    break

def left_trajectory_plot(xline, yline, zline, line_count):

    dynamic_plot = False

    # Liste elemanlarının str'den float'a dönüştürülmesini saglar.
    x = str_to_float(xline)
    y = str_to_float(yline)
    z = str_to_float(zline)

    ### TEST
    if dynamic_plot is True:
        animation_plot = ap(x,y,z, line_count)
        animation_plot.animation_function()
        print("Completed")
    else:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        plt.title('left_rokos_trajectory')
        ax.plot3D(x, y, z,'r.', label = 'Trajectory Points')
        plt.legend(loc='upper left')
        #plt.show()

def right_trajectory_plot(xline, yline, zline, line_count):

    dynamic_plot = False

    # Liste elemanlarının str'den float'a dönüştürülmesini saglar.
    x = str_to_float(xline)
    y = str_to_float(yline)
    z = str_to_float(zline)

    ### TEST
    if dynamic_plot is True:
        animation_plot = ap(x,y,z, line_count)
        animation_plot.animation_function()
        print("Completed")
    else:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        plt.title('right_rokos_trajectory')
        ax.plot3D(x, y, z,'b.', label = 'Trajectory Points')
        plt.legend(loc='upper left')
        plt.show()


def str_to_float(list):
    converted_list = []
    for i in list:
        converted_list.append(float(i))
    return converted_list

get_coordinate_datas()