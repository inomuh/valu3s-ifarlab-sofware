#define ROBOTFORLINUXTEST
#ifdef ROBOTFORLINUXTEST


#include "include/SIRRobot/KawasakiRS005LRobot.h"
#include "include/SIRRobot/SIRLinConnection.h"
#include "include/SIRRobot/SIRTypeDefs.h"
#include <iostream>
#include <vector>

using namespace std;



int main() {

  SIRMatrix P_1(6,1);
  SIRMatrix P_2(6,1);
  SIRMatrix P_3(6,1);
  SIRMatrix P_4(6,1);
  SIRMatrix P_5(6,1);
  SIRMatrix P_6(6,1);
  SIRMatrix P_7(6,1);
  SIRMatrix P_8(6,1);
  SIRMatrix P_9(6,1);
  SIRMatrix P_10(6,1);

  P_1 << 49.372, 7.036, -73.665, 163.224, 56.039, -78.017;
  P_2 << 38.955, 8.307, -76.321, 121.894, 55.717, -57.077;
  P_3 << 28.538, 9.577, -78.977, 80.563, 55.395, -36.138;
  P_4 << 18.120, 10.848, -81.633, 39.233, 55.072, -15.199;
  P_5 << 7.703, 12.119, -84.289, -2.097, 54.750, 5.740;
  P_6 << -2.714, 13.390, -86.945, -43.427, 54.428, 26.680;
  P_7 << -13.132, 14.661, -89.601, -84.757, 54.105, 47.619;
  P_8 << -23.549, 15.931, -92.257, -126.088, 53.783, 68.558;
  P_9 << -33.966, 17.202, -94.913, -167.418, 53.461, 89.498;
  P_10 << -44.383, 18.473, -97.569, -208.748, 53.139, 110.437;


  SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
  SIRConnection *con = new SIRLinConnection(logger,"192.168.5.141",11111);
  //SIRConnection *con = new SIRLinConnection(logger,"127.0.0.1",7777);
  //con->setBlockingMode(1);
  KawasakiRS005LRobot robot(con, logger,nullptr, MPT_JOINT, MT_P2P);

  if (!robot.Connect()) {
    cout << "Could not connect to robot..." << endl;
    return 0;
  }
  else
    cout << "Connected..." << endl;

  //********* CONNECTED **************
  con->setBlockingMode(0);
  robot.setWaitForCommand(2000);

  //--------------------
  cout << "send packet id: " << robot.add(P_1) << endl;
  cout << "send packet id: " << robot.add(P_2) << endl;
  cout << "send packet id: " << robot.add(P_3) << endl;
  cout << "send packet id: " << robot.add(P_4) << endl;
  cout << "send packet id: " << robot.add(P_5) << endl;
  cout << "send packet id: " << robot.add(P_6) << endl;
  cout << "send packet id: " << robot.add(P_7) << endl;
  cout << "send packet id: " << robot.add(P_8) << endl;
  cout << "send packet id: " << robot.add(P_9) << endl;
  cout << "send packet id: " << robot.add(P_10) << endl;


  sleep(10);


  if (robot.move())
    cout << "the robot is moving" << endl;
  else
    cout << "the robot is not moving" << endl;

  //hareketin bitimini bekle...
  while(robot.getStatus()!=RS_STOP){
    sleep(1);
  }

  if (robot.close())
    cout << "The connection is succesfully closed..." << endl;
  else
    cout << "The connection is not succesfully closed..." << endl;

  //**********************************

}

#endif

