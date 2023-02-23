#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <StdVector>
#include <filesystem>
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <cmath>


sensor_msgs::JointState output_msg;


double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}


void inputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

    if (msg->data.size() == 6) {

        std::vector<double> values;
        for (int i = 0; i < 6; i++) {
            double radians = degreesToRadians(msg->data[i]);
            values.push_back(radians);
        }

        //values.push_back(msg->data[0]);
        //values.push_back(msg->data[1]);
        //values.push_back(msg->data[2]);
        //values.push_back(msg->data[3]);
        //values.push_back(msg->data[4]);
        //values.push_back(msg->data[5]);
    
        std::cout << "Received values: ";
        for (const auto& value : values) {
            std::cout << value << ", ";
        }

        output_msg.header.stamp = ros::Time::now();
        output_msg.position = values;
    } 
    else {
        std::cout << "Error: message does not contain 6 elements" << std::endl;
    }

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "right_rokos_joint_publisher");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_poses", 1000, inputCallback);
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("right_rokos/joint_states", 1000);


    ros::Rate loop_rate(30);
    while (ros::ok()){

        std::cout <<"Joints == "<< output_msg<<std::endl;
        pub.publish(output_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}