#include "SIRRobotPacketPackager.h"

std::string SIRRobotPacketPackager::package(int command, void *arg)
{
	switch (command) {
	case SIRRobotCommands::CLOSE:
		return packCLOSE();
	case SIRRobotCommands::ADDTS:
		return packADDTS(static_cast<SIRMatrix*>(arg));
	case SIRRobotCommands::ADDJT:
		return packADDJT(static_cast<SIRMatrix*>(arg));
	case SIRRobotCommands::POSET:
		return packPOSET();
	case SIRRobotCommands::MOVEP:
		return packMOVEP();
	case SIRRobotCommands::POSEJ:
			return packPOSEJ();
	case SIRRobotCommands::MOVEL:
		return packMOVEL();
	case SIRRobotCommands::CLEAR:
		return packCLEAR();
	case SIRRobotCommands::MOVES:
		return packMOVES();
  case SIRRobotCommands::SIGNR:
    return packSIGNR(static_cast<int*>(arg));
  case SIRRobotCommands::SIGNW:
    return packSIGNW(static_cast<int*>(arg));
	default:
		(*logger) << SIRLOGTYPE::LOG_ERROR << "SIRRobotPacketPackager::package: Invalid command type";
		return "ERR";
	}
}

std::string SIRRobotPacketPackager::packCLOSE()
{
	return "CLOSE:";
}

std::string SIRRobotPacketPackager::packCLEAR()
{
	return "CLEAR:";
}

std::string SIRRobotPacketPackager::packADDTS(SIRMatrix *arg)
{
	packetStr.str("");
	packetStr << "ADDTS";
	for (int i = 0;i < 6;i++) {
		packetStr << " " << (*arg)(i,0);
	}
	packetStr << ":";
	return packetStr.str();
}

std::string SIRRobotPacketPackager::packADDJT(SIRMatrix *arg)
{
	packetStr.str("");
	packetStr << "ADDJT";
	for (int i = 0;i < 6;i++) {
		packetStr << " " << (*arg)(i, 0);
	}
	packetStr << ":";
	return packetStr.str();
}

std::string SIRRobotPacketPackager::packPOSET()
{
	return "POSET:";
}

std::string SIRRobotPacketPackager::packPOSEJ()
{
	return "POSEJ:";
}

std::string SIRRobotPacketPackager::packMOVEP()
{
	return "MOVEP:";
}

std::string SIRRobotPacketPackager::packMOVEL()
{
	return "MOVEL:";
}

std::string SIRRobotPacketPackager::packMOVES()
{
	return "MOVES:";
}

std::string SIRRobotPacketPackager::packSIGNR(int *arg)
{
  packetStr.str("");
  packetStr << "SIGNR "<<(*arg)<<":";
  return packetStr.str();
}

std::string SIRRobotPacketPackager::packSIGNW(int *arg)
{
  packetStr.str("");
  packetStr << "SIGNW "<<(*arg)<<":";
  return packetStr.str();
}
