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

using namespace std;
  SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
  SIRConnection *con = new SIRLinConnection(logger,"192.168.3.7",11111);
  KawasakiRS005LRobot robot(con, logger,nullptr, MPT_JOINT, MT_SCAN);
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

// Check matching of position and fposes
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
    // std::cout<< req.path << std::endl;

    std::ifstream file(req.path);
    std::string line;
    std::vector<std::vector<std::string>> data;

    if(!pathExists(req.path)) //Exits the program and outputs this message if the file is not found
    {
        // std::cout << "File not found." << std::endl;
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
        // std::cout << "OTA has arrived" << std::endl;
        state = OTA_STATE_OPERATE;
      }
      else{
        // std::cout << "Waiting for OTA arrival" << std::endl;
      }

    }
    if(state==OTA_STATE_OPERATE){
        if (!robot.Connect()) {
            // std::cout << "Could not connect to robot..." << endl;
            return 0;
        }
        else
            // std::cout << "Connected..." << endl;
        //********* CONNECTED **************

        con->setBlockingMode(0);
        robot.setWaitForCommand(2000);
        //sleep(2);

        SIRMatrix poses(6,1);
        SIRMatrix last_poses(6,1);
        SIRMatrix current_pose(6,1);
        int count = 0;

        // std::cout<< data.size() << std::endl;

        for (int i = 0; i < data.size(); ++i) {
            float pose1 = rad2deg(std::stof(data[i][0]));
            float pose2 = rad2deg(std::stof(data[i][1]));
            float pose3 = rad2deg(std::stof(data[i][2]));
            float pose4 = rad2deg(std::stof(data[i][3]));
            float pose5 = rad2deg(std::stof(data[i][4]));
            float pose6 = rad2deg(std::stof(data[i][5]));

            poses<<pose1, pose2, pose3, pose4, pose5, pose6;
            // std::cout<< pose1 <<" "<< pose2 <<" "<< pose3 <<" "<< pose4 <<" "<< pose5 <<" "<< pose6 << std::endl;
            // std::cout<< poses << std::endl;
            // last_poses = poses;
            robot.add(poses);
        }

        robot.setSpeed(9.0);

        if (robot.move())
            // std::cout << "the robot is moving" << endl;
            int filler2 = 1;
        else{
            // std::cout << "the robot is not moving" << endl;
            res.status = "abort";
            robot.close();
            return true;
        }

        //MEOZKAN-BEGIN
        bool HOLD = false;
        //MEOZKAN-END

		bool retry_connection = false;
        bool emg_stop = false;
        bool endTask = false;
        
        // std::cout <<"Before while"<<endl;
        //hareketin bitimini bekle...
        while (ros::ok()){
        // while (((robot.getStatus()!=RS_STOP) && (!HOLD)) || (HOLD)){
            
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

            // cout <<"Joints == "<< msg<<endl;
            JointPosePublisher.publish(msg);

            // Get robot status
            auto robot_status = robot.getStatus(); // RS_STOP 0 RS_MOVE 1 RS_UNKNOWN 2
            // std::cout <<"RS STATUS = "<< robot_status <<endl;

			// Check Emergency conditions
            // UI Emergency Stop
            if (((dist_listener.danger_level == 1)||((uiEmergency==1)))&&(HOLD==false)){
                robot.hold();
                HOLD = true;
                // std::cout <<"____UI HOLD____"<<std::endl;
            }

            // Button Emergency Stop
            if (((buttonEmergency==1))&&(emg_stop==false)){
                emg_stop = true;
                // std::cout <<"------Emergency STOP------"<<endl;
            }

            // Continue System
            if(((dist_listener.danger_level == 0)&&((uiEmergency!=1) && (buttonEmergency==0)))&&(HOLD==true)&&(emg_stop==false)){
                robot.cont();
                uiEmergency = 2;
                HOLD = false;
                // std::cout <<"____UI CONTINUE____"<<std::endl;
            }

            // Reconnect
            if (((emg_stop==true) && (retry_connection==false)) || (robot_status==RS_UNKNOWN)){
                if (!robot.Connect()){
                    // std::cout<<"------CONNECTION FAILED------"<< endl;
                    continue;
                }
                else{
                    // std::cout<<"------CONNECTED------"<< endl;
                    con->setBlockingMode(0);
                    retry_connection = true;
                }
            }

            // Emergecy Continue
            if(((dist_listener.danger_level == 0)&&((uiEmergency==0) && (buttonEmergency==0)))&&(emg_stop==true)&&(retry_connection==true)){
                // std::cout <<"------Emergency CONTINUE------"<<endl;
                
                for (int i = 0; i < data.size(); ++i){
                    float pose1 = rad2deg(std::stof(data[i][0]));
                    float pose2 = rad2deg(std::stof(data[i][1]));
                    float pose3 = rad2deg(std::stof(data[i][2]));
                    float pose4 = rad2deg(std::stof(data[i][3]));
                    float pose5 = rad2deg(std::stof(data[i][4]));
                    float pose6 = rad2deg(std::stof(data[i][5]));   
                    poses<<pose1, pose2, pose3, pose4, pose5, pose6;
                    // std::cout<< pose1 <<" "<< pose2 <<" "<< pose3 <<" "<< pose4 <<" "<< pose5 <<" "<< pose6 << std::endl;
                    // std::cout<< poses << std::endl;
                    robot.add(poses);
                    }

                if (robot.move()){
                    emg_stop = false;
                    HOLD = false;
                    // std::cout << "the robot is moving again" << endl;
                    uiEmergency = 2;
                }
                else{
                    // std::cout << "the robot is not moving after emergency stop" << endl;
                    continue;
                    // res.status = "abort";
                    // robot.close();
                    // return true;
                }
            }

		
            // std::cout <<"HOLD = "<< HOLD<<endl;
            
            // cout <<"CHECK == "<< checkCorrectStop(last_poses, joint_pose)<<endl;

            // Check RS Positions
            float pose1 = rad2deg(std::stof(data[0][0]));
            float pose2 = rad2deg(std::stof(data[0][1]));
            float pose3 = rad2deg(std::stof(data[0][2]));
            float pose4 = rad2deg(std::stof(data[0][3]));
            float pose5 = rad2deg(std::stof(data[0][4]));
            float pose6 = rad2deg(std::stof(data[0][5]));
            current_pose<<pose1, pose2, pose3, pose4, pose5, pose6;

            // Check current position matching then remove passed positions
            if (checkCorrectStop(current_pose, joint_pose) == true){
                if (data.size() > 1){
                    data.erase(data.begin());
                }
                else{
                    endTask = true;
                }
                
            }

            // std::cout <<"Data Size = "<< data.size()<<endl;
            
            // Task Completed
            if ((robot_status==RS_STOP) && (!HOLD) && (uiEmergency!=1) && (buttonEmergency==0) && (endTask == true)){
                // std::cout<<"------EXIT------"<< endl;
                break;
            }
            // std::cout <<"UI = "<< uiEmergency <<endl;
            // std::cout <<"Button = "<< buttonEmergency <<endl;
            // std::cout <<"------"<<endl;

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

        if (robot.close())
            // std::cout << "The connection is succesfully closed..." << endl;
            int filler = 1;
        else{
            // std::cout << "The connection is not succesfully closed..." << endl;
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
    // std::cout<<"heyyyyyyyyyyyyy"<<std::endl;
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
        // std::cout << "The connection is succesfully closed..." << endl;
    ros::spin();

    return 0;
}

#endif
