#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Bool.h"


std::vector<float> ir_arr;
float min_ir;

void irCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}
void ir1Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}
void ir2Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}
void ir3Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}
void ir4Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}
void ir5Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}
void ir6Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}
void ir7Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}
void ir8Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ir_arr.push_back(msg->range);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Publisher light_curtain_pub = n.advertise<std_msgs::Bool>("light_curtain", 1000);
  std_msgs::Bool bool_msg;
  ros::Subscriber sub = n.subscribe("ir", 1000, irCallback);
  ros::Subscriber sub1 = n.subscribe("ir1", 1000, ir1Callback);
  ros::Subscriber sub2 = n.subscribe("ir2", 1000, ir2Callback);
  ros::Subscriber sub3 = n.subscribe("ir3", 1000, ir3Callback);
  ros::Subscriber sub4 = n.subscribe("ir4", 1000, ir4Callback);
  ros::Subscriber sub5 = n.subscribe("ir5", 1000, ir5Callback);
  ros::Subscriber sub6 = n.subscribe("ir6", 1000, ir6Callback);
  ros::Subscriber sub7 = n.subscribe("ir7", 1000, ir7Callback);
  ros::Subscriber sub8 = n.subscribe("ir8", 1000, ir8Callback);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (ir_arr.size() > 8){
        min_ir = *std::min_element(ir_arr.begin(), ir_arr.end());
        ir_arr.clear();
    }
    if (min_ir >= 1.213)
        bool_msg.data = false;
    else
        bool_msg.data = true;
    light_curtain_pub.publish(bool_msg);
    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
  return 0;
}