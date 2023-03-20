#define RS005L_CONTROLLER_NODE
#ifdef RS005L_CONTROLLER_NODE

#include "include/SIRRobot/KawasakiRS005LRobot.h"
#include "include/SIRRobot/SIRLinConnection.h"
#include "include/SIRRobot/SIRTypeDefs.h"
#include <iostream>
#include <vector>
#include "eigen3/Eigen/Geometry"

//#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Int32.h"

#include <ros/ros.h>

using namespace std;

inline double SIGN(double x) {
  return (x >= 0.0) ? +1.0 : -1.0;
}
inline double NORM(double a, double b, double c, double d) {
  return sqrt(a * a + b * b + c * c + d * d);
}
// quaternion = [w, x, y, z]'
SIRMatrix mRot2Quat(const SIRMatrix& m) {
  double r11 = m(0, 0);
  double r12 = m(0, 1);
  double r13 = m(0, 2);
  double r21 = m(1, 0);
  double r22 = m(1, 1);
  double r23 = m(1, 2);
  double r31 = m(2, 0);
  double r32 = m(2, 1);
  double r33 = m(2, 2);
  double q0 = (r11 + r22 + r33 + 1.0) / 4.0;
  double q1 = (r11 - r22 - r33 + 1.0) / 4.0;
  double q2 = (-r11 + r22 - r33 + 1.0) / 4.0;
  double q3 = (-r11 - r22 + r33 + 1.0) / 4.0;
  if (q0 < 0.0) {
    q0 = 0.0;
  }
  if (q1 < 0.0) {
    q1 = 0.0;
  }
  if (q2 < 0.0) {
    q2 = 0.0;
  }
  if (q3 < 0.0) {
    q3 = 0.0;
  }
  q0 = sqrt(q0);
  q1 = sqrt(q1);
  q2 = sqrt(q2);
  q3 = sqrt(q3);
  if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
    q0 *= +1.0;
    q1 *= SIGN(r32 - r23);
    q2 *= SIGN(r13 - r31);
    q3 *= SIGN(r21 - r12);
  }
  else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
    q0 *= SIGN(r32 - r23);
    q1 *= +1.0;
    q2 *= SIGN(r21 + r12);
    q3 *= SIGN(r13 + r31);
  }
  else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
    q0 *= SIGN(r13 - r31);
    q1 *= SIGN(r21 + r12);
    q2 *= +1.0;
    q3 *= SIGN(r32 + r23);
  }
  else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
    q0 *= SIGN(r21 - r12);
    q1 *= SIGN(r31 + r13);
    q2 *= SIGN(r32 + r23);
    q3 *= +1.0;
  }
  else {
    printf("coding error\n");
  }
  double r = NORM(q0, q1, q2, q3);
  q0 /= r;
  q1 /= r;
  q2 /= r;
  q3 /= r;

  SIRMatrix res(1,4);
  res<< q0, q1, q2, q3;
  return res;
}

class distListener
{
public:
  void safetyCallback(const std_msgs::Int32::ConstPtr& s_msg);
  int danger_level;
};


vector<SIRMatrix> pathPlanner(void){

    vector<SIRMatrix> path;

    SIRMatrix sol(6,1);
    sol<<-516, 620, 345, 90, 92, -85;
    path.push_back(sol);

    SIRMatrix sag(6,1);
    sag<<484, 620, 345, 90, 92, -85;
    path.push_back(sag);

    
    // For calibration
    SIRMatrix calib1(6,1);
    SIRMatrix calib2(6,1);
    SIRMatrix calib3(6,1);
    SIRMatrix calib4(6,1);
    SIRMatrix calib5(6,1);
    SIRMatrix calib6(6,1);
    SIRMatrix calib7(6,1);
    SIRMatrix calib8(6,1);
    SIRMatrix calib9(6,1);
    SIRMatrix calib10(6,1);
    SIRMatrix calib11(6,1);
    SIRMatrix calib12(6,1);
    SIRMatrix calib13(6,1);
    SIRMatrix calib14(6,1);
    SIRMatrix calib15(6,1);

    calib1<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib2<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib3<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib4<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib5<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib6<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib7<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib8<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib9<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib10<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib11<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib12<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib13<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib14<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;
    calib15<<-521.082, 455.705, 410.794, 95.469, 122.031, 101.391;

    //path.push_back(calib1);

    return path;

}


void distListener::safetyCallback(const std_msgs::Int32::ConstPtr& s_msg)
{

  danger_level = s_msg->data;

  // std::cout << " Danger level: " << danger_level << std::endl;

}


int main(int argc, char* argv[]) {

    //ROS init
    ros::init(argc, argv, "rs005l_joint_state_publisher");
    ros::NodeHandle node;
    ros::NodeHandle pub;
    ros::NodeHandle subh;
    ros::Rate loop_rate(10);

    ros::Time stopTime;

    ros::Publisher RS005L_joint_state_publisher = pub.advertise<sensor_msgs::JointState>("/joint_states", 10);

    distListener dist_listener;
    ros::Subscriber sub = subh.subscribe("danger", 100, &distListener::safetyCallback, &dist_listener);


    //Construct the transform publishers
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    sensor_msgs::JointState jointState;


    jointState.name.push_back("joint1");
    jointState.name.push_back("joint2");
    jointState.name.push_back("joint3");
    jointState.name.push_back("joint4");
    jointState.name.push_back("joint5");
    jointState.name.push_back("joint6");


    for (int i=0;i<6;i++)
        jointState.position.push_back(0.0);



    SIRMatrix Pos(6,1);
    double J1,J2,J3,J4,J5,J6;
    double l1=0.105, l2=0.380, l3=0.143, l4=0.267;  //link length


    //Robot transform matrices
    SIRMatrix A1(4,4), A2(4,4), A3(4,4), A4(4,4), A5(4,4), A6(4,4);
    SIRMatrix quad(1,4);


    SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
    SIRConnection *con = new SIRLinConnection(logger,"192.168.5.141",11111);
    KawasakiRS005LRobot robot(con, logger,nullptr, MPT_TASK, MT_LINEAR);

    if (!robot.Connect()) {
        cout << "Could not connect to robot..." << endl;
        return 0;
    }
    else
        cout << "Connected..." << endl;
    //********* CONNECTED **************
    con->setBlockingMode(0);
    robot.setWaitForCommand(2000);

    sleep(2);

    vector<SIRMatrix> path = pathPlanner();

    for(int i=0;i<2;i++){
        cout << "send packet id: " << robot.add(path[i]) << endl;
    }

    if (robot.move())
            cout << "the robot is moving" << endl;
        else
            cout << "the robot is not moving" << endl;
    
    int flag = 0;


    while (ros::ok()){

      if(flag==0){

      if(robot.getJointPose(&Pos))
        cout << "Current Pose: " << Pos.transpose() << endl;
      else
        cout << "could not receive current pose" << endl;

      jointState.header.stamp = ros::Time::now();
     
      J1=Pos(0,0)*3.141592/180;
      J2=Pos(1,0)*3.141592/180;
      J3=Pos(2,0)*3.141592/180;
      J4=Pos(3,0)*3.141592/180;
      J5=Pos(4,0)*3.141592/180;
      J6=Pos(5,0)*3.141592/180;



      jointState.position[0] = J1;
      jointState.position[1] = J2;
      jointState.position[2] = J3;
      jointState.position[3] = J4;
      jointState.position[4] = J5;
      jointState.position[5] = J6;


      RS005L_joint_state_publisher.publish(jointState); // publish the joint states


      cout<<J1<<", "<<J2<<", "<<J3<<", "<<J4<<", "<<J5<<", "<<J6<<endl;

      }

      if((dist_listener.danger_level == 1)&&(flag==0)){
        robot.hold();
	stopTime = ros::Time::now();
	cout << "The robot stopped at " << stopTime << endl;
	flag = 1;
      }

      if((dist_listener.danger_level == 0)&&(flag==1)){
	robot.cont();
	flag = 0;
      }
      
        ros::spinOnce();

        loop_rate.sleep();

    }


    if (robot.close())
        cout << "The connection is succesfully closed..." << endl;
    else
        cout << "The connection is not succesfully closed..." << endl;

}

#endif
