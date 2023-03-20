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
  	case SIRRobotCommands::HOLD1:
   		return packHOLD1();
  	case SIRRobotCommands::CONT1:
    	return packCONT1();
  	case SIRRobotCommands::ADDJO:
   		return packADDJO(static_cast<SIRMatrix*>(arg));
  	case SIRRobotCommands::MOVEO:
    	return packMOVEO();
  	case SIRRobotCommands::DUMY1:
    	return packDUMY1();
  	case SIRRobotCommands::DUMY2:
    	return packDUMY2();
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

std::string SIRRobotPacketPackager::packHOLD1()
{
  return "HOLD1:";
}

std::string SIRRobotPacketPackager::packCONT1()
{
  return "CONT1:";
}

std::string SIRRobotPacketPackager::packADDJO(SIRMatrix *arg)
{
	packetStr.str("");
	packetStr << "ADDJO";
	for (int i = 0;i < 6;i++) {
		packetStr << " " << (*arg)(i, 0);
	}
	packetStr << ":";
	return packetStr.str();
}

std::string SIRRobotPacketPackager::packMOVEO()
{
  return "MOVEO:";
}

std::string SIRRobotPacketPackager::packDUMY1()
{
  return "DUMY1:";
}

std::string SIRRobotPacketPackager::packDUMY2()
{
  return "DUMY2:";
}
