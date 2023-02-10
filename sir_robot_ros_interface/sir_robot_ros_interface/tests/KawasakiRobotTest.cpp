//#define ROBOTTEST
#ifdef ROBOTTEST


#include "KawasakiRS005LRobot.h"
#include "SIRWinConnection.h"
#include "SIRTypeDefs.h"
#include <iostream>
#include <unistd.h>
using namespace std;


int main() {
	SIRMatrix initialPos(6, 1);
	SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
	SIRConnection *con = new SIRWinConnection(logger,"192.168.5.141",11111);
	//con->setBlockingMode(0);
	KawasakiRS005LRobot robot(con, logger,0, MPT_TASK, MT_LINEAR);

	if (!robot.Connect()) {
		cout << "Could not connect to robot..." << endl;
		return 0;
	}
	else
		cout << "Connected..." << endl;
	//********* CONNECTED **************
	robot.setWaitForCommand(2000);
	SIRMatrix pos(6,1);
//	pos << 100.555, 50.567, 1.43, 30.345, 45, 90;
//	cout<<"send packet id: "<<robot.add(pos)<<endl;

	if (robot.getTaskPose(&pos))
		cout << "Current Pose: " << pos.transpose() << endl;
	else
		cout << "could not receive current pose" << endl;

	SIRMatrix pos1(6, 1), pos2(6, 1);
	pos1 << -4.000, 600.000, 15.000, 90.000, 90.000, -90.000;
	pos2 << -4.000, 600.000, 125.000, 90.000, 90.000, -90.000;
	cout << "send packet id: " << robot.add(pos1) << endl;
	cout << "send packet id: " << robot.add(pos2) << endl;
	if (robot.move())
		cout << "the robot is moving" << endl;
	else
		cout << "the robot is not moving" << endl;

	/*Sleep(10000);
	pos2 << -4.000, 600.000, 125.000, 90.000, 90.000, -90.000;
	cout << "send packet id: " << robot.add(pos2) << endl;
	if (robot.move())
		cout << "the robot is moving" << endl;
	else
		cout << "the robot is not moving" << endl;*/

	usleep(5000);


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

	usleep(5000);

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
	
if (!robot.disconnect())
		cout << "Could not disconnect to robot..." << endl;
	else
		cout << "Disconnected..." << endl;
	return 0;
}

#endif

