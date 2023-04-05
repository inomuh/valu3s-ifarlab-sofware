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

int uiEmergency = 0;
int buttonEmergency = 0;
int connection_state = 0;

using namespace std;
  SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
  SIRConnection *con = new SIRLinConnection(logger,"192.168.3.7",11111);
  KawasakiRS005LRobot robot(con, logger,nullptr, MPT_JOINT, MT_P2P);
  //KawasakiRS005LRobot robot(con, logger,nullptr, MPT_TASK, MT_LINEAR);

  int cancel_data = 0;
  

//MEOZKAN - BEGIN
  class distListener{
  public:
    void safetyCallback(const std_msgs::Int32::ConstPtr& s_msg);
    int danger_level;
  };

  void distListener::safetyCallback(const std_msgs::Int32::ConstPtr& s_msg)
  {
    danger_level = s_msg->data;
    // std::cout << " Danger level: " << danger_level << std::endl;
  }

  distListener dist_listener;
  ros::Subscriber sub;
//MEOZKAN - END


void cancelCallback(const std_msgs::Int8::ConstPtr& msg)
{
    cancel_data = msg->data;
}

void uiEmergencyCallback(const std_msgs::Int8::ConstPtr& msg){
    uiEmergency = msg->data;
}

void buttonEmergencyCallback(const std_msgs::Int8::ConstPtr& msg){
    buttonEmergency = msg->data;
}

OTA_STATE state = OTA_STATE_WAIT;

float rad2deg(float radian){
    float degree = radian * (180.0 / M_PI);
    return degree;
}

bool checkCorrectStop(SIRMatrix poses, SIRMatrix pos){
    float th = 1.0;
    int wrong_position_count = 0;
    for (int i = 0; i < 6; i++) {
      if ((abs(abs(poses(i)) - abs(pos(i))) > th)){
        wrong_position_count = wrong_position_count + 1;
      }
    }
    if (wrong_position_count > 0){
      return false;
    }
    else{
      return true;
    }
}

bool pathExists(const std::string& path) {
    std::ifstream file(path.c_str());
    return file.good();
}


bool add(sir_robot_ros_interface::ManipulatorPose_ino_2::Request  &req,
         sir_robot_ros_interface::ManipulatorPose_ino_2::Response &res)
{
    std::cout<< req.path << std::endl;

    std::ifstream file(req.path);
    std::string line;
    std::vector<std::vector<std::string>> data;

    if(!pathExists(req.path)) //Exits the program and outputs this message if the file is not found
    {
        std::cout << "File not found." << std::endl;
        res.status = "NoFile";
        robot.close();
        return true;
    }
    ros::Rate loop_rate(15);

    std::getline(file, line); // Read and discard the first line

    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<std::string> row;

        while (std::getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }

        data.push_back(row);

    }

    file.close();

    if(state==OTA_STATE_WAIT){
      if (data.empty() == false){
        std::cout << "OTA has arrived" << std::endl;
        state = OTA_STATE_OPERATE;
      }
      else{
        std::cout << "Waiting for OTA arrival" << std::endl;
      }

    }
    if(state==OTA_STATE_OPERATE){
        if (!robot.Connect()) {
            cout << "Could not connect to robot..." << endl;
            return 0;
        }
        else
            cout << "Connected..." << endl;
        //********* CONNECTED **************

        con->setBlockingMode(0);
        robot.setWaitForCommand(2000);
        //sleep(2);


        SIRMatrix poses(6,1);
        SIRMatrix last_poses(6,1);
        int count = 0;

        std::cout<< data.size() << std::endl;

        for (int i = 0; i < data.size(); ++i) {
            float pose1 = rad2deg(std::stof(data[i][0]));
            float pose2 = rad2deg(std::stof(data[i][1]));
            float pose3 = rad2deg(std::stof(data[i][2]));
            float pose4 = rad2deg(std::stof(data[i][3]));
            float pose5 = rad2deg(std::stof(data[i][4]));
            float pose6 = rad2deg(std::stof(data[i][5]));

            poses<<pose1, pose2, pose3, pose4, pose5, pose6;
            std::cout<< pose1 <<" "<< pose2 <<" "<< pose3 <<" "<< pose4 <<" "<< pose5 <<" "<< pose6 << std::endl;
            // std::cout<< poses << std::endl;
            last_poses = poses;
            robot.add(poses);
        }

        // for (auto const& pose_list : req.poses) {
        //     if ((pose_list.pose.size() % 6) != 0){
        //         std::cout << pose_list.pose.size() << " on item " << count <<std::endl;
        //         res.status = "Wrong Input Lenght";
        //         return true;
        //     }

        //     SIRMatrix poses(6,1);
        //     poses<<pose_list.pose[0], pose_list.pose[1], pose_list.pose[2], pose_list.pose[3], pose_list.pose[4], pose_list.pose[5];
        //     last_poses = poses;
        //     count++;
        //     robot.add(poses);
        // }

        robot.setSpeed(9.0);

        if (robot.move())
            cout << "the robot is moving" << endl;
        else{
            cout << "the robot is not moving" << endl;
            res.status = "abort";
            robot.close();
            return true;
        }

        //MEOZKAN-BEGIN
        bool HOLD = false;
        //MEOZKAN-END

        bool STOPPED = false;
        
        cout <<"Before while"<<endl;
        //hareketin bitimini bekle...
        while (ros::ok()){
          //(((robot.getStatus()!=RS_STOP) && (!HOLD)) || (HOLD)){
            
          // Publish Joint States
          // TODO Not working if started on terminal&&
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

            // cout <<"Joints == "<< msg<<endl;

            JointPosePublisher.publish(msg);

            //MEOZKAN-BEGIN
            // if((dist_listener.danger_level == 1)&&(HOLD==false)){
            //     robot.hold();
            //     HOLD = true;
            // }
            // if((dist_listener.danger_level == 0)&&(HOLD==true)){
            //     robot.cont();
            //     HOLD = false;
            // }

            // Get robot status
            auto robot_status = robot.getStatus(); // RS_STOP 0 RS_MOVE 1 RS_UNKNOWN 2
            cout <<"STATUS = "<< robot_status <<endl;

            // Check Emergency conditions
            if(((dist_listener.danger_level == 1)||((uiEmergency==1)||(buttonEmergency==1)))&&(HOLD==false)){
                robot.hold();
                HOLD = true;
                cout <<"------HOLD------"<<endl;
            }
            if(((dist_listener.danger_level == 0)&&((uiEmergency==0) && (buttonEmergency==0)))&&(HOLD==true)){
                // robot.cont();
                for (int i = 0; i < data.size(); ++i) {
                    float pose1 = rad2deg(std::stof(data[i][0]));
                    float pose2 = rad2deg(std::stof(data[i][1]));
                    float pose3 = rad2deg(std::stof(data[i][2]));
                    float pose4 = rad2deg(std::stof(data[i][3]));
                    float pose5 = rad2deg(std::stof(data[i][4]));
                    float pose6 = rad2deg(std::stof(data[i][5]));

                    poses<<pose1, pose2, pose3, pose4, pose5, pose6;
                    std::cout<< pose1 <<" "<< pose2 <<" "<< pose3 <<" "<< pose4 <<" "<< pose5 <<" "<< pose6 << std::endl;
                    // std::cout<< poses << std::endl;
                    last_poses = poses;
                    robot.add(poses);
                }
                robot.move();
                HOLD = false;
                cout <<"------CONTINUE------"<<endl;
            }
            cout <<"HOLD = "<< HOLD<<endl;
            
            // Emergency button pressed robot connection needed to be refreshed
            if ((buttonEmergency==1) && (robot_status==RS_STOP) && (connection_state==0)){
              connection_state = 1;
              // robot.close();
              cout <<"------CLOSE CONNECTION------"<<endl;
            }

            // Emergecy button released
            if (((buttonEmergency==0) && (connection_state==1)) || (robot_status==RS_UNKNOWN)){
              if (!robot.Connect()){
                cout<<"------CONNECTION FAILED------"<< endl;
                continue;
              }
              else{
                cout<<"------CONNECTED------"<< endl;
                connection_state = 0;
              }
            }

            
            // if (!(((robot_status!=RS_STOP) && (!HOLD)) || (HOLD))){
            //   cout<<"------EXIT------"<< endl;
            //   break;
            // }
            cout <<"CHECK == "<< checkCorrectStop(last_poses, joint_pose)<<endl;
            // Task Completed
            if ((robot_status==RS_STOP) && (!HOLD) && (uiEmergency==0) && (checkCorrectStop(last_poses, joint_pose) == true)){
              cout<<"------EXIT------"<< endl;
              break;

            }
            cout <<"------"<<endl;

            //MEOZKAN-END
          //  if (robot.getStatus()==RS_STOP && !flag){
          //      break;
          //  }
            //robot.setSignal(1,true);
            
           // robot.setSignal(1,false);

            //cout<<"service "<<dist_listener.danger_level<<endl;
            ros::spinOnce();
            loop_rate.sleep();
        }

        cout <<"after while"<<endl;
        SIRMatrix pos(6,1);
        robot.getJointPose(&pos);
        if (checkCorrectStop(last_poses, pos) == false){
            cout << "stopped at wrong position" << endl;
            res.status = "abort";
            robot.close();
            return true;
        }


    //    if (robot.setSignal(1,false))
    //        cout << "success to set signal..." << endl;
    //    else{
    //        cout << "fail to set signal...4" << endl;
    //        res.status = "abort";
    //        robot.close();
    //        return true;
    //    }
        //sleep(2);

        if (robot.close())
            cout << "The connection is succesfully closed..." << endl;
        else{
            cout << "The connection is not succesfully closed..." << endl;
            res.status = "abort";
            robot.close();
            return true;
        }
        //**********************************
        state = OTA_STATE_FINISH;

    }

    if(state == OTA_STATE_FINISH){
        ros::Duration(1.0).sleep();
        state = OTA_STATE_WAIT;
        res.status = "succeed";
    }
  return true;
}


int main(int argc, char **argv) {


  //SIRConnection *con = new SIRLinConnection(logger,"127.0.0.1",7777);
  //con->setBlockingMode(1);

  ros::init(argc, argv, "khi_ota_comm");

  ros::NodeHandle n;

  JointPosePublisher = n.advertise<std_msgs::Float64MultiArray>("joint_poses", 1000);
  ros::ServiceServer service = n.advertiseService("manipulator_service", add);
  ros::Subscriber sub = n.subscribe("manipulator/cancel", 1000, cancelCallback);
  ros::Subscriber sub2 = n.subscribe("ui_emg", 1000, uiEmergencyCallback);
  ros::Subscriber sub3 = n.subscribe("emg", 1000, buttonEmergencyCallback);

//MEOZKAN - BEGIN
  sub = n.subscribe("danger", 100, &distListener::safetyCallback, &dist_listener);
  std::cout<<"heyyyyyyyyyyyyy"<<std::endl;
//MEOZKAN - END
  robot.Connect();
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
  for(int i=0;i<10;i++){
    JointPosePublisher.publish(msg);
    sleep(1);
  }
  if (robot.close())
    cout << "The connection is succesfully closed..." << endl;
  ros::spin();

  return 0;
}

#endif

