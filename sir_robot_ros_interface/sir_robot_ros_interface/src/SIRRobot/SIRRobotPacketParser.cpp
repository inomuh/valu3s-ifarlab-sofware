#include "SIRRobotPacketParser.h"
#include "SIRRobotCommands.h"

int SIRRobotPacketParser::parse(const std::string &packet, void *data)
{
	size_t found = packet.find(':');
	if (found == std::string::npos) {
		(*logger) << SIRLOGTYPE::LOG_ERROR << "SIRRobotPacketParser::parse: Invalid packet type";
		return -1;
	}
	packetStr.clear();
	packetStr<<packet.substr((size_t)0,found);
	packetStr >> packStr;
	switch (SIRRobotCommands::getID(packStr)) {
	case SIRRobotCommands::CLOSE:
		return parseCLOSE();
	case SIRRobotCommands::ADDTS:
		return parseADDTS(static_cast<int *>(data));
	case SIRRobotCommands::ADDJT:
		return parseADDJT(static_cast<int *>(data));
	case SIRRobotCommands::POSET:
		return parsePOSET(static_cast<SIRMatrix *>(data));
	case SIRRobotCommands::POSEJ:
		return parsePOSEJ(static_cast<SIRMatrix *>(data));
	case SIRRobotCommands::MOVEP:
		return parseMOVEP();
	case SIRRobotCommands::MOVEL:
		return parseMOVEL();
	case SIRRobotCommands::CLEAR:
		return parseCLEAR();
	case SIRRobotCommands::MOVES:
		return parseMOVES();
  	case SIRRobotCommands::SIGNR:
    	return parseSIGNR(static_cast<bool*>(data));
 	case SIRRobotCommands::SIGNW:
   	 	return parseSIGNW();
 	case SIRRobotCommands::HOLD1:
  		return parseHOLD1();
  	case SIRRobotCommands::CONT1:
   		return parseCONT1();
  	case SIRRobotCommands::ADDJO:
    	return parseADDJO();
  	case SIRRobotCommands::MOVEO:
   		return parseMOVEO();
  	case SIRRobotCommands::DUMY1:
    	return parseDUMY1();
  	case SIRRobotCommands::DUMY2:
    	return parseDUMY2();
	default:
		(*logger) << SIRLOGTYPE::LOG_ERROR << "SIRRobotPacketParser::parse: Invalid command type";
		return -1;
	}
}

int SIRRobotPacketParser::parseCLOSE()
{
	return SIRRobotCommands::CLOSE;
}

int SIRRobotPacketParser::parseCLEAR()
{
	return SIRRobotCommands::CLEAR;
}

int SIRRobotPacketParser::parseADDTS(int *id)
{
	(*id) = 0;
	packetStr >> (*id);
	return SIRRobotCommands::ADDTS;
}

int SIRRobotPacketParser::parseADDJT(int *id)
{
	(*id) = 0;
	packetStr >> (*id);
	return SIRRobotCommands::ADDJT;
}

int SIRRobotPacketParser::parsePOSET(SIRMatrix *pos)
{
	packetStr >> (*pos)(0,0)>> (*pos)(1, 0)>> (*pos)(2, 0)>> (*pos)(3, 0)>> (*pos)(4, 0)>> (*pos)(5, 0);
	return SIRRobotCommands::POSET;
}

int SIRRobotPacketParser::parseMOVEP()
{
	return SIRRobotCommands::MOVEP;
}

int SIRRobotPacketParser::parseMOVEL()
{
	return SIRRobotCommands::MOVEL;
}

int SIRRobotPacketParser::parsePOSEJ(SIRMatrix *pos)
{
	packetStr >> (*pos)(0, 0) >> (*pos)(1, 0) >> (*pos)(2, 0) >> (*pos)(3, 0) >> (*pos)(4, 0) >> (*pos)(5, 0);
	return SIRRobotCommands::POSEJ;
}

int SIRRobotPacketParser::parseMOVES()
{
	return SIRRobotCommands::MOVES;
}

int SIRRobotPacketParser::parseSIGNR(bool *sig)
{
  packetStr >> (*sig);
  return SIRRobotCommands::SIGNR;
}

int SIRRobotPacketParser::parseSIGNW()
{
  return SIRRobotCommands::SIGNW;
}

int SIRRobotPacketParser::parseHOLD1()
{
  return SIRRobotCommands::HOLD1;
}

int SIRRobotPacketParser::parseCONT1()
{
  return SIRRobotCommands::CONT1;
}

int SIRRobotPacketParser::parseADDJO()
{
  return SIRRobotCommands::ADDJO;
}

int SIRRobotPacketParser::parseMOVEO()
{
  return SIRRobotCommands::MOVEO;
}

int SIRRobotPacketParser::parseDUMY1()
{
  return SIRRobotCommands::DUMY1;
}

int SIRRobotPacketParser::parseDUMY2()
{
  return SIRRobotCommands::DUMY2;
}


