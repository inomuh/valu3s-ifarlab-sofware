//#define PACKETTEST
#ifdef PACKETTEST

#include "SIRRobotPacketPackager.h"
#include "SIRRobotPacketParser.h"
#include "SIRWinConnection.h"
#include "SIRTypeDefs.h"
#include <iostream>
using namespace std;

int main() {
	SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
	SIRConnection *con = new SIRWinConnection(logger);
	SIRPacketParser *parser = new SIRRobotPacketParser(logger);
	SIRPacketPackager *packer = new SIRRobotPacketPackager(logger);
/*
	if (!con->Connect()) {
		cout << "Could not connect to robot..." << endl;
		return 0;
	}
	else
		cout << "Connected..." << endl;
		*/
	//********* CONNECTED **************
	string str;
	int id;
	SIRMatrix pos(6, 1);
	pos << 100.555, 50.567, 1.43, 30.345, 45, 90;
	str=packer->package(SIRRobotCommands::ADDTS, &pos);
	cout << "Send packet: " << str << endl;

	str = packer->package(SIRRobotCommands::POSET);
	cout << "Send packet: " << str << endl;

	str = packer->package(SIRRobotCommands::CLOSE);
	cout << "Send packet: " << str << endl;

	str = packer->package(SIRRobotCommands::MOVEP);
	cout << "Send packet: " << str << endl;

	str = "POSET 100.555 50.567 1.43 -30.345 -45 90:";
	cout << "Received command: " << SIRRobotCommands::getCommand(parser->parse(str,&pos)) << endl;
	cout << pos << endl;
	
	str = "ADDTS 34:";
	cout << "Received command: " << SIRRobotCommands::getCommand(parser->parse(str, &id)) << endl;
	cout << id << endl;

	str = "CLOSE:";
	cout << "Received command: " << SIRRobotCommands::getCommand(parser->parse(str)) << endl;

	str = "MOVEP:";
	cout << "Received command: " << SIRRobotCommands::getCommand(parser->parse(str)) << endl;

	str = "MOVEP :";
	cout << "Received command: " << SIRRobotCommands::getCommand(parser->parse(str)) << endl;

	str = "MOVEPP:";
	cout << "Received command: " << SIRRobotCommands::getCommand(parser->parse(str)) << endl;

	str = "MOVEP";
	cout << "Received command: " << SIRRobotCommands::getCommand(parser->parse(str)) << endl;

	//**********************************
	/*
	if (!con->disconnect())
		cout << "Could not disconnect to robot..." << endl;
	else
		cout << "Disconnected..." << endl;
		*/
	return 0;
}

#endif
