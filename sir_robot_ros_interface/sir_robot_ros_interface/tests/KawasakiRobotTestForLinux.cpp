#define ROBOTFORLINUXTEST
#ifdef ROBOTFORLINUXTEST


#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"
#include "khi_ota_comm.h"
#include <iostream>
#include <vector>
#include <StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SIRMatrix)

using namespace std;




void otaListener::khiOtaCommCallback(const agv_msgs::TaskCom::ConstPtr& s_msg)
{

  s_station_id = s_msg->station_id;
  s_product_name = s_msg->product_name;
  s_product_count = s_msg->product_count; 
  s_duty = s_msg->duty; 
  s_status = s_msg->status;

  /*std::cout << " Station id: " << s_station_id << std::endl;
  std::cout << " Product Name: " << s_product_name << std::endl;
  std::cout << " Product Count: " << s_product_count << std::endl;
  std::cout << " Duty: " << s_duty << std::endl;
  std::cout << " Status: " << s_status << std::endl;
  std::cout << " -------------------- " << std::endl;*/

}


OTA_STATE state = OTA_STATE_WAIT;



vector<SIRMatrix> pathPlanner(string boxType){

    vector<SIRMatrix> path;

    SIRMatrix P_Top_Box(6,1);
    SIRMatrix P_Top_Vehicle(6,1);
    SIRMatrix P_Home(6,1);

    SIRMatrix P_Grip(6,1);     //200x200x200
    SIRMatrix P_Before_Grip(6,1);
    SIRMatrix P_Before_Drop(6,1);
    SIRMatrix P_Drop(6,1);

    SIRMatrix P_Grip2(6,1);    //250x250x350
    SIRMatrix P_Before_Grip2(6,1); //
    SIRMatrix P_Before_Drop2(6,1);
    SIRMatrix P_Drop2(6,1);




    P_Top_Box<<-694.890, 82.273, 313.291, 85.340, 178.828, 153.062;
    P_Top_Vehicle<<28.606, 700.716, 313.287, 85.366, 178.828, 153.090;
    P_Home<<-14.793, 291.511, 479.387, 85.348, 178.828, 153.068;


    P_Grip<<-693.328, 201.249, -91.597, 85.320, 178.827, 153.041;
    P_Before_Grip<<-693.328, 201.249, 275.292, 85.320, 178.827, 153.041;

    P_Before_Drop<<330.606, 700.716, 275.287, 85.366, 178.828, 153.090;
    P_Drop<<330.606, 700.716, -100.445, 85.271, 178.828, 152.991;

    P_Grip2<<-743.907, -54.142, 10.640, 85.337, 178.829, 153.058;
    P_Before_Grip2<<-746.907, -54.142, 275.292, 85.337, 178.829, 153.058;

    P_Before_Drop2<<0.606, 700.716, 275.287, 85.366, 178.828, 153.090;
    P_Drop2<<0.606, 700.716, -5.445, 85.271, 178.828, 152.991;



    // B is for small box
    if(boxType == "B"){

        path.push_back(P_Home);
        path.push_back(P_Top_Box);
        path.push_back(P_Before_Grip);
        path.push_back(P_Grip);
        // Grip process
        path.push_back(P_Before_Grip);
        path.push_back(P_Top_Box);
        path.push_back(P_Top_Vehicle);
        path.push_back(P_Before_Drop);
        path.push_back(P_Drop);
        // Drop process
        path.push_back(P_Before_Drop);
        path.push_back(P_Top_Vehicle);
        path.push_back(P_Home);

    }


    // C is for large box
    if(boxType == "C"){

        path.push_back(P_Home);
        path.push_back(P_Top_Box);
        path.push_back(P_Before_Grip2);
        path.push_back(P_Grip2);
        // Grip process
        path.push_back(P_Before_Grip2);
        path.push_back(P_Top_Box);
        path.push_back(P_Top_Vehicle);
        path.push_back(P_Before_Drop2);
        path.push_back(P_Drop2);
        // Drop process
        path.push_back(P_Before_Drop2);
        path.push_back(P_Top_Vehicle);
        path.push_back(P_Home);

    }

    return path;


}



int main(int argc, char **argv) {



    SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
    SIRConnection *con = new SIRLinConnection(logger,"192.168.3.7",11111);
    //SIRConnection *con = new SIRLinConnection(logger,"127.0.0.1",7777);
    //con->setBlockingMode(1);
    KawasakiRS005LRobot robot(con, logger,nullptr, MPT_TASK, MT_LINEAR);











  ros::init(argc, argv, "khi_ota_comm");

  ros::NodeHandle n;

  otaListener ota_listener;
  ros::Subscriber sub = n.subscribe("ota_status", 1000, &otaListener::khiOtaCommCallback, &ota_listener);

  ros::Publisher pub = n.advertise<agv_msgs::TaskCom>("rk_hmi_status", 1000);
  agv_msgs::TaskCom p_msg;

  ros::Rate loop_rate(10);



  while (ros::ok())
  {

    if(state==OTA_STATE_WAIT){

      if((ota_listener.s_station_id == 1)&&(ota_listener.s_status == 1)){
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



        sleep(2);


        vector<SIRMatrix> path = pathPlanner(ota_listener.s_product_name);



        //open Gripper
        if (robot.setSignal(1,false))
            cout << "success to set signal..." << endl;
        else
            cout << "fail to set signal..." << endl;
        if (robot.setSignal(2,false))
            cout << "success to set signal..." << endl;
        else
            cout << "fail to set signal..." << endl;
        if (robot.setSignal(1,true))
            cout << "success to set signal..." << endl;
        else
            cout << "fail to set signal..." << endl;


        for(int i=0;i<4;i++)
        {
            cout << "send packet id: " << robot.add(path[i]) << endl;
        }



        if (robot.move())
            cout << "the robot is moving" << endl;
        else
            cout << "the robot is not moving" << endl;

        //hareketin bitimini bekle...
        while(robot.getStatus()!=RS_STOP){
            sleep(1);
        }


        //Tutma yap
        if (robot.setSignal(1,false))
            cout << "success to set signal..." << endl;
        else
            cout << "fail to set signal..." << endl;
        sleep(1);
        if (robot.setSignal(2,true))
            cout << "success to set signal..." << endl;
        else
            cout << "fail to set signal..." << endl;
        //Tutma bekle
        sleep(2);


        for(int i=4;i<9;i++)
        {
            cout << "send packet id: " << robot.add(path[i]) << endl;
        }



        if (robot.move())
            cout << "the robot is moving" << endl;
        else
            cout << "the robot is not moving" << endl;

        //hareketin bitimini bekle...
        while(robot.getStatus()!=RS_STOP){
            sleep(1);
        }




        //bırak yap
        if (robot.setSignal(2,false))
            cout << "success to set signal..." << endl;
        else
            cout << "fail to set signal..." << endl;
        sleep(1);
        if (robot.setSignal(1,true))
            cout << "success to set signal..." << endl;
        else
            cout << "fail to set signal..." << endl;
        //bırak bekle
        sleep(2);





        for(int i=9;i<12;i++)
        {
            cout << "send packet id: " << robot.add(path[i]) << endl;
        }



        if (robot.move())
            cout << "the robot is moving" << endl;
        else
            cout << "the robot is not moving" << endl;

        //hareketin bitimini bekle...
        while(robot.getStatus()!=RS_STOP){
            sleep(1);
        }




        if (robot.setSignal(1,false))
            cout << "success to set signal..." << endl;
        else
            cout << "fail to set signal..." << endl;


        sleep(2);


        if (robot.close())
            cout << "The connection is succesfully closed..." << endl;
        else
            cout << "The connection is not succesfully closed..." << endl;
        //**********************************

        state = OTA_STATE_FINISH;

    }



    if(state == OTA_STATE_FINISH){


          p_msg.station_id = 1;
          p_msg.product_name = "B";
          p_msg.product_count = 1;
          p_msg.duty = 1;
          p_msg.status = 1;

          for(int i=0;i<10;i++){
            pub.publish(p_msg);
          }

          ros::Duration(5.0).sleep();

          state = OTA_STATE_WAIT;

        }

    ros::spinOnce();

    loop_rate.sleep();



    }


  ros::spin();



  return 0;

}

#endif

