#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convery xml file input to yaml file

@Author: Furkan Edizkan
@Task: #IFARLAB
"""

import xml.etree.ElementTree as ET
import yaml
import sys
import os


def readXml(path):
    """Read xml file"""
    tree = ET.parse(path)
    root = tree.getroot()
    task_list = []
    for task in root:
        task_dict = {}
        position_dict = {}
        for child in task:
            if child.tag == "Task_ID" or child.tag == "Mode":
                task_dict[child.tag] = int(child.text)
            if child.tag == "Vehicle_CODE" or child.tag == "Tag":
                task_dict[child.tag] = str(child.text)
            if child.tag == "Position":
                for pose in child:
                    position_dict[pose.tag] = float(pose.text)
                task_dict[child.tag] = position_dict
        task_list.append(task_dict)
    return task_list


def writeYaml(path, task_list, header):
    """Write yaml file"""    
    right_rokos = [{"Task": {
        "Task_ID": item["Task_ID"],
        "Vehicle_CODE": item["Vehicle_CODE"],
        "Mode": item["Mode"],
        "Tag": item["Tag"],
        "Position": {
            "X": item["Position"]["X"],
            "Y": item["Position"]["Y"],
            "Z": item["Position"]["Z"],
            "roll": item["Position"]["roll"],
            "pitch": item["Position"]["pitch"],
            "yaw": item["Position"]["yaw"]
            }
        }} for item in task_list]
    yaml_dict = {header: right_rokos}
    with open(path, "w") as file:
        file.write("---\n\n")
        yaml.dump(yaml_dict, file)
    print("# {}.yaml created.".format(header))


def checkInputLen(inputs):
    """Check input length"""
    if (inputs in ("", None, [])) or len(inputs) != 1:
        print("# Wrong number of arguments")
        print("  Please provide correct arguments")
        print("  Example: ...python3 ...xml_yaml.py <XML File Path>")
        print("---")
        return False
    #print("Correct input length")
    return True


def checkPath(path):
    """Check if path exists"""
    if not(os.path.exists(path)):
        print("# Path: || {} || does not exist".format(path))
        print("  Please provide correct path for XML file")
        print("---")
        return False
    #print("File exists")
    return True


def checkType(path):
    """Check if input type correct"""
    if not (path.lower().endswith(('.xml'))):
        print("# Wrong file type")
        print("  Please provide correct file type")
        print("---")
        return False
    #print("File format is XML")
    return True


def checkOutputDir(yaml_dir_path, header):
    """Check if output directory exists"""
    if not (os.path.isdir(yaml_dir_path)):
        print("# Output directory does not exist")
        print("  Creating a yaml directory under path: " + yaml_dir_path)
        os.mkdir(yaml_dir_path)
    yaml_path = os.path.join(yaml_dir_path, header + ".yaml")
    return yaml_path
    

def checkInputs():
    """Check inputs"""
    inputs = sys.argv[1:]
    if not (checkInputLen(inputs)):
        raise SystemExit
    path = inputs[0]
    if not (checkPath(path) and checkType(path)):
        raise SystemExit
    return path


def main():
    """Main function"""
    
    yaml_dir_path = "/home/ifarlab/catkin_ws/src/mobile_manipulator/srvt_moveit/params/"
    header = "Right_Rokos"
    try:
        xml_path = checkInputs()
        task_list = readXml(path=xml_path)
        yaml_path = checkOutputDir(yaml_dir_path, header)
        writeYaml(path=yaml_path, task_list=task_list, header=header)
    except Exception as main_err:
        print("Error::main::{}".format(main_err))
    
    
    
if __name__ == "__main__":
    main()

"""
Run this for testing purposes
/usr/bin/env python3 "/home/furkaned/catkin_ws/src/utilities/Scripts/xml_yaml_converter/xml_yaml.py" /home/furkaned/catkin_ws/src/utilities/Scripts/xml_yaml_converter/xml/left_rokos_task.xml

"""
