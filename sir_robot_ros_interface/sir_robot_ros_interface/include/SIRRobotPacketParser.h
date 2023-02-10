#ifndef SIRROBOTPACKETPARSER_H
#define SIRROBOTPACKETPARSER_H
#include "SIRPacketParser.h"
#include "SIRTypeDefs.h"

//! SIR Robot Communication Packet parser class
/*!
This class is used to parse a packet for communication with robot into the data 
*/
class SIRRobotPacketParser : public SIRPacketParser{
public:
	SIRRobotPacketParser(SIRLogger *log):SIRPacketParser(log){}
	
	/** Parses a communication packet into the data
	* @param packet the received packet to be parsed
	* @param data parsed data which may be different types according to command type.
	* @return returns the type of the command if no error; otherwise returns -1.
	*/
	int parse(const std::string &packet, void *data);
private:
	/** Parses the CLOSE packet
	* @return returns the type of the packet
	*/
	int parseCLOSE();

	/** Parses the CLEAR packet
	* @return returns the type of the packet
	*/
	int parseCLEAR();

	/** Parses the ADDTS packet
	* @param id the number of waypoints which are sent
	* @return returns the type of the packet
	*/
	int parseADDTS(int *id);

	/** Parses the ADDJT packet
	* @param id the number of waypoints which are sent
	* @return returns the type of the packet
	*/
	int parseADDJT(int *id);

	/** Parses the POSET packet
	* @param pos the current TCP position of the robot
	* @return returns the type of the packet
	*/
	int parsePOSET(SIRMatrix *pos);

	/** Parses the POSEJ packet
	* @param pos the current joint angles of the robot
	* @return returns the type of the packet
	*/
	int parsePOSEJ(SIRMatrix *pos);

	/** Packs the MOVEP packet
	* @return returns the type of the packet
	*/
	int parseMOVEP();

	/** Packs the MOVEL packet
	* @return returns the type of the packet
	*/
	int parseMOVEL();

	/** Packs the MOVES packet
	* @return returns the type of the packet
	*/
	int parseMOVES();

  /** Parses the SIGNR packet
  * @param sig the current value of the signal (on:true, off:false)
  * @return returns the type of the packet
  */
  int parseSIGNR(bool *sig);

  /** Parses the SIGNW packet
  * @return returns the type of the packet
  */
  int parseSIGNW();
};

#endif
