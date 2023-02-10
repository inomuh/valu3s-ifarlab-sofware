#ifndef SIRPACKETPACKAGER_H
#define SIRPACKETPACKAGER_H
#include <string>
#include <sstream>
#include "SIRLogger.h"
//! SIR Communication Packet packager abstract class
/*!
This class is used to wrap the data as a communication packet
*/
class SIRPacketPackager{
public:
	SIRPacketPackager(SIRLogger *log):logger(log){}

	/** Packs the data as a communication packet 
	* @param command type of the command
	* @param arg packet argument which may be different types according to command type.
	* @return returns the packet which is ready to send.
	*/
  virtual std::string package(int command, void *arg=nullptr) = 0;
protected:
	std::string packStr;
	std::stringstream packetStr;
	SIRLogger *logger;
};

#endif
