# VALUE3S IFARLAB Software


## Features
- Gazebo Simulation - Chassis analysis simulation environment with a robotic arm
- Planning and motion controller codes
- Web UI - Robot platform and task management UI
- Kawasaki RS005L - ROS conenction


## Requirements
- Ubuntu 20.04
- ROS Noetic
- Python 3.8 & General Python Libraries
- MoveIt v1.1.11 - Inside Repository
- ros_control v0.19.6 - Inside Repository
- ros_controller v 0.21.0 - Inside Repository

# Installation

Clone this repostiory inside ROS workspace

```bash
cd ~<user>/<workspace>/src/
git clone https://github.com/inomuh/valu3s-ifarlab-sofware.git
```

Install dependencies with "rosdep install"
```bash
rosdep install --from-paths src --ignore-src -r -y
```

Return to workspace folder then build the environment with "catkin_make"
```bash
cd ~<user>/<workspace>/
catkin_make
```

Source gazebo setup add gazebo models inside .bashrc

```bash
gedit ~/.bashrc
```

Add following lines to end of file

- ***source /usr/share/gazebo-11/setup.sh***

- ***export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/***



# How to Start 

#### Gazebo Simulation Control Start
```bash
roslaunch srvt_moveit start_simulation_system.launch
```

#### Real Robot Control Start

!!! Needs a real robot conenction to work !!!
```bash
roslaunch srvt_moveit start_simulation_system.launch
```

#### Web UI Start
```bash
~<user>/<workspace>/src/mobile_manipulator/ifarlab_web/index_v2.html
```

# The Contributors
- [Didem Özüpek Taş](https://github.com/DidemOzupekTas) 
- [Furkan Edizkan](https://github.com/FurkanEdizkan)

## License

See [LICENSE-APACHE](LICENSE-APACHE) and [LICENSE-MIT](LICENSE-MIT) for details.
