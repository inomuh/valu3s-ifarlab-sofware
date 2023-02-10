//#define ROBOTFORLINUXTESTONTABLE
#ifdef ROBOTFORLINUXTESTONTABLE


#include "include/SIRRobot/KawasakiRS005LRobot.h"
#include "include/SIRRobot/SIRLinConnection.h"
#include "include/SIRRobot/SIRTypeDefs.h"
#include <iostream>
#include <vector>

using namespace std;



int main() {
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


  P_Grip<<-693.328, 201.249, -81.597, 85.320, 178.827, 153.041;
  P_Before_Grip<<-693.328, 201.249, 275.292, 85.320, 178.827, 153.041;

  P_Before_Drop<<330.606, 700.716, 275.287, 85.366, 178.828, 153.090;
  P_Drop<<330.606, 700.716, -100.445, 85.271, 178.828, 152.991;

  P_Grip2<<-746.907, -54.142, 10.640, 85.337, 178.829, 153.058;
  P_Before_Grip2<<-746.907, -54.142, 275.292, 85.337, 178.829, 153.058;

  P_Before_Drop2<<0.606, 700.716, 275.287, 85.366, 178.828, 153.090;
  P_Drop2<<0.606, 700.716, -5.445, 85.271, 178.828, 152.991;


  SIRMatrix initialPos(6, 1);
  SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
  SIRConnection *con = new SIRLinConnection(logger,"192.168.5.141",11111);
  //SIRConnection *con = new SIRLinConnection(logger,"127.0.0.1",7777);
  //con->setBlockingMode(1);
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
  SIRMatrix pos(6,1);
//  pos << -296.994,  171.916,  107.011,  -86.317,  175.986,  -80.247;
//	cout<<"send packet id: "<<robot.add(pos)<<endl;

  if (robot.getTaskPose(&pos))
    cout << "Current Pose: " << pos.transpose() << endl;
  else
    cout << "could not receive current pose" << endl;

//  SIRMatrix pos1(6, 1), pos2(6, 1);
//   pos1 << -296.994,  171.916,  115.011,  -86.317,  175.986,  -80.247;
//  pos1 << -4.000, 600.000, 15.000, 90.000, 90.000, -90.000;
//  pos2 << -4.000, 600.000, 125.000, 90.000, 90.000, -90.000;

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

  //First Box
  //--------------------
  cout << "send packet id: " << robot.add(P_Home) << endl;
  cout << "send packet id: " << robot.add(P_Top_Box) << endl;
  cout << "send packet id: " << robot.add(P_Before_Grip) << endl;
  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;
  //hareketin bitimini bekle...
  while(robot.getStatus()!=RS_STOP){
    sleep(1);
  }

  cout << "send packet id: " << robot.add(P_Grip) << endl;
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
  cout << "send packet id: " << robot.add(P_Before_Grip) << endl;
  cout << "send packet id: " << robot.add(P_Top_Box) << endl;
  cout << "send packet id: " << robot.add(P_Top_Vehicle) << endl;
  cout << "send packet id: " << robot.add(P_Before_Drop) << endl;
  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;
  //hareketin bitimini bekle...
  while(robot.getStatus()!=RS_STOP){
    sleep(1);
  }
  cout << "send packet id: " << robot.add(P_Drop) << endl;
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
  cout << "send packet id: " << robot.add(P_Before_Drop) << endl;
  cout << "send packet id: " << robot.add(P_Top_Vehicle) << endl;
  //cout << "send packet id: " << robot.add(P_Home) << endl;
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

//  ROBOTSTATUS  status;
//  status=robot.getStatus();
//  cout<<"Robot status: "<<((status==RS_MOVE)?"moving":((status==RS_STOP)?"stop":"unknown"))<<endl;

  //-------------------------

  //Second Box
  //--------------------
  //cout << "send packet id: " << robot.add(P_Home) << endl;
  cout << "send packet id: " << robot.add(P_Top_Box) << endl;
  cout << "send packet id: " << robot.add(P_Before_Grip2) << endl;
  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;
  //hareketin bitimini bekle...
  while(robot.getStatus()!=RS_STOP){
    sleep(1);
  }

  cout << "send packet id: " << robot.add(P_Grip2) << endl;
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
  cout << "send packet id: " << robot.add(P_Before_Grip2) << endl;
  cout << "send packet id: " << robot.add(P_Top_Box) << endl;
  cout << "send packet id: " << robot.add(P_Top_Vehicle) << endl;
  cout << "send packet id: " << robot.add(P_Before_Drop2) << endl;
  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;
  //hareketin bitimini bekle...
  while(robot.getStatus()!=RS_STOP){
    sleep(1);
  }
  cout << "send packet id: " << robot.add(P_Drop2) << endl;
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
  cout << "send packet id: " << robot.add(P_Before_Drop2) << endl;
  cout << "send packet id: " << robot.add(P_Top_Vehicle) << endl;
  cout << "send packet id: " << robot.add(P_Home) << endl;
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

//  ROBOTSTATUS  status;
//  status=robot.getStatus();
//  cout<<"Robot status: "<<((status==RS_MOVE)?"moving":((status==RS_STOP)?"stop":"unknown"))<<endl;

  //-------------------------




//  if (robot.move())
//    cout << "the robot is moving" << endl;
//  else
//    cout << "the robot is not moving" << endl;

  sleep(2);
//  status=robot.getStatus();
//  cout<<"Robot status: "<<((status==RS_MOVE)?"moving":((status==RS_STOP)?"stop":"unknown"))<<endl;

  //----- Signal TEst-----------
//  sleep(2);
//  if (robot.setSignal(13,true))
//    cout << "success to set signal..." << endl;
//  else
//    cout << "fail to set signal..." << endl;
//  bool sigValue;
//  if (robot.getSignal(13,sigValue))
//    cout << "Signal 1:" << ((sigValue)?"ON":"OFF")<<endl;
//  else
//    cout << "fail to get signal..." << endl;
//sleep(5);
//  if (robot.setSignal(13,false))
//    cout << "success to set signal..." << endl;
//  else
//    cout << "fail to set signal..." << endl;

//  if (robot.getSignal(13,sigValue))
//    cout << "Signal 1:" << ((sigValue)?"ON":"OFF")<<endl;
//  else
//    cout << "fail to get signal..." << endl;
   //----- Signal TEst-----------



//  char ch;
//  cout<<"After motion, press a key.... ";
//  cin>>ch;
sleep(2);
  /*Sleep(10000);
  pos2 << -4.000, 600.000, 125.000, 90.000, 90.000, -90.000;
  cout << "send packet id: " << robot.add(pos2) << endl;
  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;*/

//  sleep(1);


  /*initialPos = pos;

  pos(0) = initialPos(0) + 250;
  cout << "send packet id: " << robot.add(pos) << endl;

  pos(0) = initialPos(0);
  pos(2) = initialPos(2) - 150;
  cout << "send packet id: " << robot.add(pos) << endl;

  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;

  Sleep(5000);

  pos(0) = initialPos(0) - 250;
  pos(2) = initialPos(2);
  cout << "send packet id: " << robot.add(pos) << endl;

  pos(0) = initialPos(0);
  pos(2) = initialPos(2);
  cout << "send packet id: " << robot.add(pos) << endl;

  robot.setMoveType(MOVETYPE::MT_P2P);

  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;*/

//  sleep(1);

  //if (robot.clear())
  //	cout << "the path points are cleared...." << endl;
  //else
  //	cout << "the path points are not cleared...." << endl;

  /*pos(1) = pos(1) - 5;
  cout << "send packet id: " << robot.add(pos) << endl;

  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;*/

  ////	cout << "send packet id: " << robot.add(pos) << endl;
//
//	cout << "send packet id: " << robot.move() << endl;
  if (robot.close())
    cout << "The connection is succesfully closed..." << endl;
  else
    cout << "The connection is not succesfully closed..." << endl;
  //**********************************
}

#endif

