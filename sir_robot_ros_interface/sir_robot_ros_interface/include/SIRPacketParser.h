#ifndef SIRPACKETPARSER_H
#define SIRPACKETPARSER_H
#include <string>
#include <sstream>
#include "SIRLogger.h"
//! SIR Communication Packet parser abstract class
/*!
This class is used to parse a communication packet
*/
class SIRPacketParser {
public:
	SIRPacketParser(SIRLogger *log) :logger(log) {}

	/** Parses a communication packet into the data
	* @param packet the received packet to be parsed
	* @param data parsed data which may be different types according to command type.
	* @return returns the type of the command.
	*/
  virtual int parse(const std::string &packet, void *data = nullptr) = 0;
protected:
	std::string packStr;
	std::stringstream packetStr;
	SIRLogger *logger;
};

#endif


