#include "SIRConnection.h"


SIRConnection::SIRConnection(SIRLogger *log, string _ip, unsigned int _port)
	:ip(_ip),port(_port),logger(log),functionCB(NULL)
{

}

SIRConnection::~SIRConnection()
{

}
