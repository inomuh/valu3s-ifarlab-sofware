#ifndef SIRROBOTPACKETPACKAGER_H
#define SIRROBOTPACKETPACKAGER_H
#include <string>
#include <sstream>
#include "SIRRobotCommands.h"
#include "SIRTypeDefs.h"
#include "SIRPacketPackager.h"

//! SIR Robot Communication Packet packager class
/*!
This class is used to wrap the data as a packet for communication with robot
*/
class SIRRobotPacketPackager:public SIRPacketPackager
{
public:
	SIRRobotPacketPackager(SIRLogger *log):SIRPacketPackager(log){}
	
	/** Packs the data as a communication packet for the robot
	* @param command type of the command
	* @param arg packet argument which may be different types according to command type.
	* @return returns the packet which is ready to send if no error; otherwise returns "ERR".
	*/
	std::string package(int command, void *arg);

private:
	/** Packs the CLOSE packet
	* @return returns the packet which is ready to send.
	*/
	std::string packCLOSE();

	/** Packs the ADDTS packet
	* @param arg the position vector for the robot TCP. The size of the vector [x y z O A T]' is (6x1) in millimeters and degrees.   
	* @return returns the packet which is ready to send.
	*/
	std::string packADDTS(SIRMatrix *arg);

	/** Packs the ADDJT packet
	* @param arg the position vector for the robot joints. The size of the vector [j1 j2 j3 j4 j5 j6]' is (6x1) in degrees.
	* @return returns the packet which is ready to send.
	*/
	std::string packADDJT(SIRMatrix *arg);

	/** Packs the POSET packet
	* @return returns the packet which is ready to send.
	*/
	std::string packPOSET();

	/** Packs the POSEJ packet
	* @return returns the packet which is ready to send.
	*/
	std::string packPOSEJ();

	/** Packs the MOVEP packet
	* @return returns the packet which is ready to send.
	*/
	std::string packMOVEP();

	/** Packs the CLEAR packet
	* @return returns the packet which is ready to send.
	*/
	std::string packCLEAR();

	/** Packs the MOVEL packet (MOVELINEAR, robot moves through the path linearly.)
	* @return returns the packet which is ready to send.
	*/
	std::string packMOVEL();

	/** Packs the MOVES packet (MOVEANDSCAN, robot moves through the path linearly, trigger the sensor and send position)
	* @return returns the packet which is ready to send.
	*/
	std::string packMOVES();

	/** Packs the STATS packet (STATUS)
	* @return returns the packet which is ready to send.
	*/
	//std::string packSTATS();

	/** Packs the SPEED packet (SPEED)
	* @return returns the packet which is ready to send.
	*/
	//std::string packSPEED();

  /** Packs the SIGNW packet
  * @param arg the number of signal to be set (minus sign for OFF, o/w ON).
  * @return returns the packet which is ready to send.
  */
  std::string packSIGNW(int* arg);

  /** Packs the SIGNR packet
  * @param arg the number of signal to be get.
  * @return returns the packet which is ready to send.
  */
  std::string packSIGNR(int* arg);

};

#endif

