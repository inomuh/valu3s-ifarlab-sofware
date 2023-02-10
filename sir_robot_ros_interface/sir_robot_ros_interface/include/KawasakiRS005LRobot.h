#ifndef KAWASAKIRS005LROBOT_H
#define KAWASAKIRS005LROBOT_H

#include "SIRConnection.h"
#include "SIRRobot.h"
#include "SIRTypeDefs.h"
#include "SIRPoseDB.h"
#include<iostream>
#include<string.h>
using std::string; 

	
//! Robot interface class
/*!
	This class is used as an API to interface Kawasaki robot.
*/
class KawasakiRS005LRobot:public SIRRobot{ //prototipleri yarat
public:
	/** Constructor
	* @param _port Port number of the connection
	* @param _ip ip number of the connection
	* @param ptype waypoint types (joint space or task space)
	* @param mtype move type for tracking waypoints
	*/
	KawasakiRS005LRobot(SIRConnection *conn, SIRLogger *log, SIRPoseDB *poseDatabase = NULL
		,MOVEPOINTTYPE ptype = MPT_JOINT, MOVETYPE mtype = MT_NONE);
	
	
	~KawasakiRS005LRobot();  // robot icin yaratýlan nesneyi yok et (destructure)
	
	/** Sends waypoints for robot motion in task or joint space 
	* @param SIRMatrix is a vector that shows the x,y,z,O,A,T for task space and J1,J2,J3,J4,J5,J6 for joint space in millimeters
	* @return the point id
	*/
	int add(SIRMatrix& pos);  

	
	/** this function is used to clear information in the robot
	* @return the result of the clear function (false: not clear, true: clear)
	*/
	bool clear();

	/** this function is used to close connection to the robot
	* @return the result of the close operation (false: not close, true: close)
	*/
	bool close();

	/** this function is used to set the speed of the robot
	* @param sp speed that will be set
	* @return the speed of the robot
	*/
	int setSpeed(int sp);


	/** this function is used to get joint pose of the robot
	* @param SIRMatrix is a vector that shows the J1,J2,J3,J4,J5,J6
	* @return the information of the whether position information available or not
	*/
	bool getJointPose(SIRMatrix *);


	/** this function is used to get task pose of the robot
	* @param SIRMatrix is a vector that shows the x,y,z,O,A,T
	* @return the information of the whether position information available or not
	*/
	bool getTaskPose(SIRMatrix *);

	/** this function is used to move the robot
	* @return the movement (True: there is a movement, False: there is not a movement )
	*/
	bool move();

	/** this function is used to check the status of the robot
	* @return the status of the robot
	*/
	virtual ROBOTSTATUS getStatus();

	/** this function is used to set the time how much wait for a packet
	* @param wait the duration in milliseconds
	*/
	void setWaitForCommand(unsigned int wait) {
		if (wait > 0)
			msWait = wait;
	}

	/** this function is a callback funtion for a robot position
	* @param msg received message from robot
	*/
	static void robotCB(string msg);

	/** this function is used to stop callback function
	*/
	void finishCB();

  /** this function is used to get signal state
  * @param sigNo signal number to get the state
  * @param sigValue the signal state (True: signal ON, false: signal OFF)
  * @return the result of the set operation (false: not get, true: success)
  */
  bool getSignal(int sigNo, bool &sigValue);

  /** this function is used to set signal state
  * @param sigNo signal number to get the state
  * @param state value to set (ON:true/OFF:false)
  * @return the result of the set operation (false: not set, true: success)
  */
  bool setSignal(int sigNo, bool state);


private:
	/**
	* Indicates the number of waypoints
	*/
	int pointID;
	std::string str;
	unsigned int msWait;

	/**
	* indicates where the poses are kept.
	*/
	static SIRPoseDB *poseDB;
	static SIRMatrix *pose;
	static SIRPacketParser *poseParser;
};

#endif 

