#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

float sin1, sin2, sin3, sin4, sin5, sin6;
float cos1, cos2, cos3, cos4, cos5, cos6;
float joint1, joint2, joint3, joint4, joint5, joint6;
float pose_x, pose_y, pose_qz, pose_qw;
float r, p, y;

using namespace std;

float** mulMat(float** mat1, float** mat2)
{
  float** rslt = new float*[4];
  for (int i = 0; i < 4; i++) {
    rslt[i] = new float[4];
    for (int j = 0; j < 4; j++) {
      rslt[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        rslt[i][j] += mat1[i][k] * mat2[k][j];
      }
    }
  }
  return rslt;
}

void showMatrix(std::string name, float** mat1)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {  
            cout << mat1[i][j] << " ";
        }
        cout << endl;
    }
}

float** zerosFunction(){
  float** arr = new float*[4];
  for (int i = 0; i < 4; ++i) {
    arr[i] = new float[4];
    for (int j = 0; j < 4; ++j) {
      arr[i][j] = 0;
    }
  }
  return arr;
}

void odomCallback(const nav_msgs::Odometry& msg)
{

  pose_x = msg.pose.pose.position.x;
  pose_y = msg.pose.pose.position.y;
  pose_qw = msg.pose.pose.orientation.w;
  pose_qz = msg.pose.pose.orientation.z;
  
}

void jointTrajCallback(const sensor_msgs::JointState& msg)
{
  if (msg.position.size()>3){
  sin1 = sin(msg.position[0]);
  sin2 = sin(msg.position[1]);
  sin3 = sin(msg.position[2]);
  sin4 = sin(msg.position[3]);
  sin5 = sin(msg.position[4]);
  sin6 = sin(msg.position[5]);
  cos1 = cos(msg.position[0]);
  cos2 = cos(msg.position[1]);
  cos3 = cos(msg.position[2]);
  cos4 = cos(msg.position[3]);
  cos5 = cos(msg.position[4]);
  cos6 = cos(msg.position[5]);
  joint1 = msg.position[0];
  joint2 = msg.position[1];
  joint3 = msg.position[2];
  joint4 = msg.position[3];
  joint5 = msg.position[4];
  joint6 = msg.position[5];
}}

float rad2deg(float radian){
 float degree = radian * (180.0 / M_PI);
 return degree;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle n;
  ros::Subscriber jointPoseSubscriber = n.subscribe("/right_rokos/joint_states", 1000, jointTrajCallback);
  ros::Subscriber odomSubscriber = n.subscribe("/odom", 1000, odomCallback);

  ros::Rate loop_rate(50);
  std::cout<<"hey"<<std::endl;
  float l0 = 0.285;
  float l1 = 0.105;
  float l2 = 0.38;
  float l3y = 0.08;
  float l3 = 0.143;
  float l4 = 0.267;
  float l5 = 0.078;
  float** A1 = zerosFunction();
  float** A2 = zerosFunction();
  float** A3 = zerosFunction();
  float** A4 = zerosFunction();
  float** A5 = zerosFunction();
  float** A6 = zerosFunction();
  static tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  tf::Quaternion q;
  q.setX(0);
  q.setY(0);
  //sleep(120);
  while (ros::ok()) {
    A1[0][0] = cos1;
    A1[0][1] = sin1;
    A1[1][0] = -sin1;
    A1[1][1] = cos1;
    A1[2][3] = l0;
    A1[2][2] = A1 [3][3] = 1;

    A2[0][2] = -1;
    A2[1][0] = sin2;
    A2[1][1] = cos2;
    A2[1][3] = l1;
    A2[2][0] = cos2;
    A2[2][1] = -sin2;
    A2[3][3] = 1;

    A3[0][0] = cos3;
    A3[0][1] = sin3;
    A3[0][3] = l2;
    A3[1][0] = -sin3;
    A3[1][1] = cos3;
    A3[2][2] = A3[3][3] = 1;

    A4[0][2] = A4[3][3] = 1;
    A4[0][3] = l3;
    A4[1][0] = sin4;
    A4[1][1] = cos4;
    A4[1][3] = -l3y;
    A4[2][0] = -cos4;
    A4[2][1] = sin4;
    
    A5[0][2] = -1;
    A5[1][0] = -sin5;
    A5[1][1] = cos5;
    A5[2][0] = cos5;
    A5[2][1] = sin5;
    A5[2][3] = l4;
    A5[3][3] = 1;

    A6[0][2] = A6[3][3] = 1;
    A6[0][3] = l5;
    A6[1][0] = sin6;
    A6[1][1] = cos6;
    A6[2][0] = -cos6;
    A6[2][1] = sin6;

    transform.setOrigin( tf::Vector3(pose_x,0.21,0.2));
    q.setRPY(0,0,0);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "ota_base_link"));


    transform.setOrigin( tf::Vector3(0,-0.26,0.212));
    q.setRPY(0,0,0);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ota_base_link", "manipulator_base_link"));

    transform.setOrigin( tf::Vector3(0,0,0.05));
    q.setRPY(0,0,-3.1415);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "manipulator_base_link", "base_link"));

    transform.setOrigin( tf::Vector3(A1[0][3], A1[1][3], A1[2][3]));
    p = atan2(-A1[2][0], sqrt(pow(A1[0][0],2) + pow(A1[1][0],2)));
    y = atan2(A1[1][0]/cos(p),A1[0][0]/cos(p));
    r = atan2(A1[2][1]/cos(p),A1[2][2]/cos(p));
    q.setRPY(r, p, y);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "link1"));

    transform.setOrigin( tf::Vector3(A2[0][3], A2[1][3], A2[2][3]));
    p = atan2(-A2[2][0], sqrt(pow(A2[0][0],2) + pow(A2[1][0],2)));
    y = atan2(A2[1][0]/cos(p),A2[0][0]/cos(p));
    r = atan2(A2[2][1]/cos(p),A2[2][2]/cos(p));
    q.setRPY(r, p, y);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link1", "link2"));
  
    transform.setOrigin( tf::Vector3(A3[0][3], A3[1][3], A3[2][3]));
    p = atan2(-A3[2][0], sqrt(pow(A3[0][0],2) + pow(A3[1][0],2)));
    y = atan2(A3[1][0]/cos(p),A3[0][0]/cos(p));
    r = atan2(A3[2][1]/cos(p),A3[2][2]/cos(p));
    q.setRPY(r, p, y);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link2", "link3"));
  
    transform.setOrigin( tf::Vector3(A4[0][3], A4[1][3], A4[2][3]));
    p = atan2(-A4[2][0], sqrt(pow(A4[0][0],2) + pow(A4[1][0],2)));
    y = atan2(A4[1][0]/cos(p),A4[0][0]/cos(p));
    r = atan2(A4[2][1]/cos(p),A4[2][2]/cos(p));
    q.setRPY(r, p, y);    
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link3", "link4"));
  
    transform.setOrigin( tf::Vector3(A5[0][3], A5[1][3], A5[2][3]));
    p = atan2(-A5[2][0], sqrt(pow(A5[0][0],2) + pow(A5[1][0],2)));
    y = atan2(A5[1][0]/cos(p),A5[0][0]/cos(p));
    r = atan2(A5[2][1]/cos(p),A5[2][2]/cos(p));
    q.setRPY(r, p, y);    
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link4", "link5"));
  
    transform.setOrigin( tf::Vector3(A6[0][3], A6[1][3], A6[2][3]));
    p = atan2(-A6[2][0], sqrt(pow(A6[0][0],2) + pow(A6[1][0],2)));
    y = atan2(A6[1][0]/cos(p),A6[0][0]/cos(p));
    r = atan2(A6[2][1]/cos(p),A6[2][2]/cos(p));
    q.setRPY(r, p, y);    
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link5", "link6"));

    transform.setOrigin( tf::Vector3(0, 0, 0));
    q.setRPY(0, 0, 2.9);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link6", "link7"));

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}

