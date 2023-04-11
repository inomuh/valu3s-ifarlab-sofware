#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;
float j_1,j_2,j_3,j_4,j_5,j_6;


void jointPoseCallback(const sensor_msgs::JointState& msg)
{
  j_1 = msg.position[0];
  j_2 = msg.position[1];
  j_3 = msg.position[2];
  j_4 = msg.position[3];
  j_5 = msg.position[4];
  j_6 = msg.position[5];
//   std::cout<<j_1<<" "<<j_2<<" "<<j_3<<" "<<j_4<<" "<<j_5<<" "<<j_6<<std::endl;
}
  

float rad2deg(float radian){
 float degree = radian * (180.0 / M_PI);
 return degree;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_s_publisher");
  ros::NodeHandle n;
  ros::Subscriber jointPoseSubscriber = n.subscribe("/right_rokos/joint_states", 1000, jointPoseCallback);
  //ros::Subscriber jointPoseSubscriber = n.subscribe("/manipulator_controller/command", 1000, jointTrajCallback);
  ros::Publisher jointCommandPublisher = n.advertise<trajectory_msgs::JointTrajectory>("/manipulator_controller/command", 1000);
  trajectory_msgs::JointTrajectory poses;
  trajectory_msgs::JointTrajectoryPoint points;
  ros::Rate loop_rate(10);
  std::vector<std::string> joint_names;
  std::cout<<"jndkfejbkj"<<std::endl;
  joint_names.push_back("joint1");
  joint_names.push_back("joint2");
  joint_names.push_back("joint3");
  joint_names.push_back("joint4");
  joint_names.push_back("joint5");
  joint_names.push_back("joint6");
  int seq;
  sleep(100);
  while (ros::ok())
  {
    poses.header.seq = seq;
    poses.header.stamp = ros::Time::now();
    poses.joint_names = joint_names;
    points.positions.push_back(j_1);
    points.positions.push_back(j_2);
    points.positions.push_back(j_3);
    points.positions.push_back(j_4);
    points.positions.push_back(j_5);
    points.positions.push_back(j_6);
    points.time_from_start = ros::Duration(0.1);
    poses.points.push_back(points);
    jointCommandPublisher.publish(poses);
    points.positions.clear();

    seq++;
    poses.points.clear();

    // std::cout<<j_1<<" "<<j_2<<" "<<j_3<<" "<<j_4<<" "<<j_5<<" "<<j_6<<std::endl;
    ros::spinOnce();

    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}
