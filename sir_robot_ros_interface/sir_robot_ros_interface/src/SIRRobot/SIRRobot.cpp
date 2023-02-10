#include "SIRRobot.h"

SIRRobot::SIRRobot(SIRConnection *conn, SIRLogger *log, MOVEPOINTTYPE ptype, MOVETYPE mtype)
	:con(conn), movePointType(ptype), moveType(mtype),status(RS_UNKNOWN), logger(log), functionCB(NULL)
{
	packer = new SIRRobotPacketPackager(logger);
	parser = new SIRRobotPacketParser(logger);

}

ROBOTSTATUS SIRRobot::getStatus()
{
	return status;
}

SIRRobot::~SIRRobot()
{
	delete packer;
	delete parser;
}

bool SIRRobot::Connect()
{
	return con->Connect();
}

bool SIRRobot::disconnect()
{
	return con->disconnect();
}

void SIRRobot::setMoveType(MOVETYPE type) 
{
	moveType = type;
}

bool SIRRobot::registerCB(void *funcCB)
{
	return false;
}
