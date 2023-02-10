#ifndef SIRROBOT_H
#define SIRROBOT_H

#include "include/SIRRobot/SIRConnection.h"
#include "include/SIRRobot/SIRRobotPacketParser.h"
#include "include/SIRRobot/SIRRobotPacketPackager.h"
#include "include/SIRLogger.h"
#include "include/SIRRobot/SIRTypeDefs.h"

//! Abstract Robot interface class
/*!
	This class is used as an API to interface an industrial robot.
*/
class SIRRobot{
protected:
	/**
	* Indicates a pointer for connection
	*/
	SIRConnection *con;
	SIRLogger *logger;
	SIRPacketParser *parser;
	SIRPacketPackager *packer;
	void *functionCB;
public:
	SIRRobot(SIRConnection *conn, SIRLogger *log, MOVEPOINTTYPE ptype = MPT_JOINT, MOVETYPE mtype = MT_NONE);
	virtual ~SIRRobot();

	/** this function is used to connection of the robot
	* @return the result of the connection process (false: not connected, true: connected)
	*/
	bool Connect();
	
	/** this function is used to disconnection of the robot
	* @return the result of the disconnection process (false: not disconnected, true: disconnected)
	*/
	bool disconnect();

	/** this function is used to check the status of the robot
	* @return the status of the robot
	*/
	virtual ROBOTSTATUS getStatus();

	/** this function is used to change move type
	* @param move type
	*/
	void setMoveType(MOVETYPE type);

	/** this function is used to define a callback function for getting positions asynchronously.
	* @param funcCB pointer for the callback function
	* @return true if success, false otherwise
	*/
	bool registerCB(void *funcCB);
protected:
	/**
	* Indicates a variable for move point type
	*/
	MOVEPOINTTYPE movePointType;

	/**
	* Indicates a variable for move type for robot
	*/
	MOVETYPE moveType;

	/**
	* Indicates the status of the robot
	*/
	ROBOTSTATUS status;
};

#endif

