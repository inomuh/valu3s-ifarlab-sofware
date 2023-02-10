#ifndef SIRROBOTCOMMANDS_H
#define SIRROBOTCOMMANDS_H
#include <string>

//! SIR Robot Communication Commands
/*!
This class is used to define commands
*/
class SIRRobotCommands {
public:
	//! Defines log level 
	/*! These levels defines which type of log messages are reported.*/
	enum ROBOTCOMMANS {
		CLOSE=0,			/*!< close the connection*/
		ADDTS,			/*!< sends a waypoint of the motion plan defined on task space (ADDTaskSpace)*/
		ADDJT,			/*!< sends a waypoint of the motion plan defined on joint space (ADDJointSpace)*/
		CLEAR,			/*!< clear the all waypoints sent before */
		MOVES,			/*!< start the motion through the waypoints sent before and request sending the TCP position continuously while moving
						     (MOVEandScan)*/
		MOVEP,			/*!< start the point-to-point motion through the waypoints sent before (MOVEPointtopoint) */
		MOVEL,			/*!< start the linear motion through the waypoints sent before (MOVELinear) */
		STATS,			/*!< request the robot status */
		SPEED,			/*!< Set the robot speed */
		POSEJ,			/*!< Request the robot position in joint space (POSEtioninJointspace) */
    POSET,			/*!< Request the robot position in task space (POSEtioninTaskspace) */
    SIGNR,      /*!< Get the signal */
    SIGNW      /*!< Set the signal */
	};

	static std::string getCommand(int command){
		switch (command) {
		case CLOSE:
			return "CLOSE";
		case ADDTS:
			return "ADDTS";
		case ADDJT:
			return "ADDJT";
		case CLEAR:
			return "CLEAR";
		case MOVES:
			return "MOVES";
		case MOVEP:
			return "MOVEP";
		case MOVEL:
			return "MOVEL";
		case STATS:
			return "STATS";
		case SPEED:
			return "SPEED";
		case POSEJ:
			return "POSEJ";
		case POSET:
			return "POSET";
    case SIGNR:
      return "SIGNR";
    case SIGNW:
      return "SIGNW";
		default:
			return "";
		}
	}
	static int getID(const std::string str) {
		if(str=="CLOSE")
			return CLOSE;
		else if (str == "ADDTS")
			return ADDTS;
		else if (str == "ADDJT")
			return ADDJT;
		else if (str == "CLEAR")
			return CLEAR;
		else if (str == "MOVES")
			return MOVES;
		else if (str == "MOVEP")
			return MOVEP;
		else if (str == "MOVEL")
			return MOVEL;
		else if (str == "STATS")
			return STATS;
		else if (str == "SPEED")
			return SPEED;
		else if (str == "POSEJ")
			return POSEJ;
		else if (str == "POSET")
			return POSET;
    else if (str == "SIGNR")
      return SIGNR;
    else if (str == "SIGNW")
      return SIGNW;
		else
			return -1;
	}
};


#endif
