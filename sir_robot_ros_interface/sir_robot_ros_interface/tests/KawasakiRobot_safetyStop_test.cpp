#define ROBOTFORLINUXTEST
#ifdef ROBOTFORLINUXTEST


#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"
#include "khi_ota_comm_valu3s.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <sir_robot_ros_interface/ManipulatorPose_ino_2.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <StdVector>
#include <filesystem>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SIRMatrix)

ros::Publisher JointPosePublisher;

using namespace std;
  SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
  SIRConnection *con = new SIRLinConnection(logger,"192.168.3.7",11111);
  KawasakiRS005LRobot robot(con, logger,nullptr, MPT_JOINT, MT_P2P);
  //KawasakiRS005LRobot robot(con, logger,nullptr, MPT_TASK, MT_LINEAR);

class distListener{
  public:
    void safetyCallback(const std_msgs::Int32::ConstPtr& s_msg);
    int danger_level=0;
  };

  void distListener::safetyCallback(const std_msgs::Int32::ConstPtr& s_msg)
  {
    danger_level = s_msg->data;
    // std::cout << " Danger level: " << danger_level << std::endl;
  }

  distListener dist_listener;
  ros::Subscriber sub;

int main(int argc, char **argv) {


  //SIRConnection *con = new SIRLinConnection(logger,"127.0.0.1",7777);
  //con->setBlockingMode(1);

  ros::init(argc, argv, "khi_ota_comm");

  ros::NodeHandle n;
 
  JointPosePublisher = n.advertise<std_msgs::Float64MultiArray>("joint_poses", 1000);
  sub = n.subscribe("danger", 100, &distListener::safetyCallback, &dist_listener);
  ros::Rate loop_rate(10);

  if (!robot.Connect()) {
  	cout << "Could not connect to robot..." << endl;
        return 0;
  }
  else
       cout << "Connected..." << endl;
  //********* CONNECTED **************

  con->setBlockingMode(0);
  robot.setWaitForCommand(2000);

  SIRMatrix poses(6,1);
  SIRMatrix last_poses(6,1);

  poses<< 83.563, 13.266, -151.778, -84.093, 88.889, -234.443; //down (1,1)
  last_poses<<83.546, -8.56, -71.088, -92.381, 84.462, -131.755; //up (1,5)
  // robot.add(poses);
  // robot.add(last_poses);
  // robot.setSpeed(9.0);

  // if (robot.move())
  //   cout << "the robot is moving" << endl;
  // else{
  //   cout << "the robot is not moving" << endl;
  //   robot.close();
  //   return true;
  // }

  bool HOLD=false;
  while(ros::ok()){
         
  	// Publish Joint States

    SIRMatrix joint_pose(6,1);
    robot.getJointPose(&joint_pose);

	  std_msgs::Float64MultiArray msg;
    msg.data.resize(6);
    msg.data[0] = joint_pose(0);
    msg.data[1] = joint_pose(1);
    msg.data[2] = joint_pose(2);
    msg.data[3] = joint_pose(3);
    msg.data[4] = joint_pose(4);
    msg.data[5] = joint_pose(5);

    cout <<"Joints (DEG)  == "<< msg<<endl;

    JointPosePublisher.publish(msg);

    // if((dist_listener.danger_level == 1)&&(HOLD==false)){
    //   robot.hold();
    //   HOLD = true;
    // }

    // if((dist_listener.danger_level == 0)&&(HOLD==true)){
    //   robot.cont();
    //   HOLD = false;
    // }    
       

    robot.setSignal(1,true);
       
	  ros::spinOnce();
        
    loop_rate.sleep();
        
     robot.setSignal(1,false);
  }
		
  if (robot.close())
        cout << "The connection is succesfully closed..." << endl;
  else{
	      cout << "The connection is not succesfully closed..." << endl;
  }

  return 0;
}

#endif

