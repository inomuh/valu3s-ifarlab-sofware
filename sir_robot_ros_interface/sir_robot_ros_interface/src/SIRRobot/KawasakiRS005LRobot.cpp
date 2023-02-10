#include <iostream>
#include <string.h>
#include "KawasakiRS005LRobot.h"   
using namespace std; 
using std::string; 

SIRPoseDB *KawasakiRS005LRobot::poseDB;
SIRPacketParser *KawasakiRS005LRobot::poseParser;
SIRMatrix *KawasakiRS005LRobot::pose = new SIRMatrix(6, 1);

KawasakiRS005LRobot::KawasakiRS005LRobot(SIRConnection *conn, SIRLogger *log, SIRPoseDB *poseDatabase, MOVEPOINTTYPE ptype, MOVETYPE mtype)
	:SIRRobot(conn, log, ptype, mtype),msWait(0)
{
	poseDB = poseDatabase;
	poseParser = parser;
} // robot icin nesne yarat (constructor)

KawasakiRS005LRobot::~KawasakiRS005LRobot()  // robot icin yaratýlan nesneyi yok et (destructure)
{}

int KawasakiRS005LRobot::add(SIRMatrix& pos)
{
	str.clear();
	void *ptr = &pos;
	int id;
	if (movePointType == MPT_JOINT) {
		if (con->Send(packer->package(SIRRobotCommands::ADDJT, ptr)) != 0) {
			if (con->Receive(str,msWait) != 0) {
				if (parser->parse(str, &id) == SIRRobotCommands::ADDJT)
					return id;
				else {
					(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::add: could not receive correct answer";
					return -1;
				}
			}
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::add: could not receive answer";
				return -1;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::add: could not send point";
			return -1;
		}
	}
	else if (movePointType == MPT_TASK) {
		if (con->Send(packer->package(SIRRobotCommands::ADDTS, ptr)) != 0) {
			if (con->Receive(str, msWait) != 0) {
				if (parser->parse(str, &id) == SIRRobotCommands::ADDTS)
					return id;
				else {
					(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::add: could not receive correct answer";
					return -1;
				}
			}
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::add: could not receive answer";
				return -1;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::add: could not send point";
			return -1;
		}
	}
	else {
		(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::add: incorrect move point type";
		return -1;
	}
}

//to be coded
bool KawasakiRS005LRobot::clear()
{
	str.clear();
	if (con->Send(packer->package(SIRRobotCommands::CLEAR)) != 0) {
		if (con->Receive(str, msWait) != 0) {
			if (parser->parse(str) == SIRRobotCommands::CLEAR) {
				return true;
			}
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::clear: could not receive correct answer";
				return false;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::clear: could not receive answer";
			return false;
		}
	}
	else {
		(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::clear: could not send close command";
		return false;
	}
	return false;
}

//to be coded
ROBOTSTATUS KawasakiRS005LRobot::getStatus()
{
  str.clear();
  int sigNo=2020; // signal number for get the value of CYCLE_START (if value:true, robot moving, o/w stop)
  void *ptr = &sigNo;
  bool sigValue;

  if (con->Send(packer->package(SIRRobotCommands::SIGNR,ptr)) != 0) {
    if (con->Receive(str, msWait) != 0) {
      if (parser->parse(str, &sigValue) == SIRRobotCommands::SIGNR){
        if(sigValue)
          return RS_MOVE;
        else {
           return RS_STOP;
        }
      }
      else {
        (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getStatus: could not receive correct answer";
        return RS_UNKNOWN;
      }
    }
    else {
      (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getStatus: could not receive answer";
      return RS_UNKNOWN;
    }
  }
  else {
    (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getStatus: could not send point";
    return RS_UNKNOWN;
  }
}

//to be coded
int KawasakiRS005LRobot::setSpeed(int speed)
{
	return speed;
}

bool KawasakiRS005LRobot::close() 
{
	str.clear();
	if (con->Send(packer->package(SIRRobotCommands::CLOSE)) != 0) {
		if (con->Receive(str, msWait) != 0) {
			if (parser->parse(str) == SIRRobotCommands::CLOSE) {
				con->disconnect();
				return true;
			}
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::close: could not receive correct answer";
				return -1;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::close: could not receive answer";
			return -1;
		}
	}
	else {
		(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::close: could not send close command";
		return -1;
	}

}

bool KawasakiRS005LRobot::getTaskPose(SIRMatrix *pos)
{
	str.clear();
	void *ptr = pos;
	int id;

	if (con->Send(packer->package(SIRRobotCommands::POSET)) != 0) {
		if (con->Receive(str, msWait) != 0) {
			if (parser->parse(str, ptr) == SIRRobotCommands::POSET)
				return true;
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getTaskPose: could not receive correct answer";
				return false;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getTaskPose: could not receive answer";
			return false;
		}
	}
	else {
		(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getTaskPose: could not send point";
		return false;
	}
}

bool KawasakiRS005LRobot::getJointPose(SIRMatrix *pos)
{
	str.clear();
	void *ptr = pos;
	int id;

	if (con->Send(packer->package(SIRRobotCommands::POSEJ)) != 0) {
		if (con->Receive(str, msWait) != 0) {
			if (parser->parse(str, ptr) == SIRRobotCommands::POSEJ)
				return true;
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getJointPose: could not receive correct answer";
				return false;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getJointPose: could not receive answer";
			return false;
		}
	}
	else {
		(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getJointPose: could not send point";
		return false;
	}
}

bool KawasakiRS005LRobot::move()
{
	str.clear();
	switch (moveType) {
	case MT_SCAN:
		if (con->Send(packer->package(SIRRobotCommands::MOVES)) != 0) {
			cout << "test1" << endl;
			if (con->Receive(str, msWait) != 0) {
				cout << "test2: " << str << endl;
				if (parser->parse(str) == SIRRobotCommands::MOVES) {
					cout << "test3" << endl;
					//Here initiate a callback function to get the positions send by robot 
					if (!con->registerCB(&robotCB)) {
						(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not registerCB";
						return false;
					}
					if (!con->startCB(con)) {
						(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not startCB";
						return false;
					}
					return true;
				}
				else {
					(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not receive correct answer";
					return false;
				}
			}
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not receive answer";
				return false;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not send command";
			return false;
		}
		return false;
	case MT_P2P:
		if (con->Send(packer->package(SIRRobotCommands::MOVEP)) != 0) {
			if (con->Receive(str, msWait) != 0) {
				if (parser->parse(str) == SIRRobotCommands::MOVEP)
					return true;
				else {
					(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not receive correct answer";
					return false;
				}
			}
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not receive answer";
				return false;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not send command";
			return false;
		}
	case MT_LINEAR:
		if (con->Send(packer->package(SIRRobotCommands::MOVEL)) != 0) {
			if (con->Receive(str, msWait) != 0) {
				if (parser->parse(str) == SIRRobotCommands::MOVEL)
					return true;
				else {
					(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not receive correct answer";
					return false;
				}
			}
			else {
				(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not receive answer";
				return false;
			}
		}
		else {
			(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: could not send command";
			return false;
		}
		return false;
	case MT_CIRCULAR:
		//to be coded
		return false;
	case MT_NONE:
		//to be coded
		return false;
	default:
		(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::move: there is an error.";
		return false;
	}
}

void KawasakiRS005LRobot::robotCB(string msg)
{
	void *ptr = pose;
	if (poseParser->parse(msg, ptr) == SIRRobotCommands::POSET) {
		poseDB->pushData((*pose));
	}
//	cout << msg << endl;
}

void KawasakiRS005LRobot::finishCB()
{
	if (!con->stopCB()) {
		(*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::finishCB: could not stop CB";
	}
}

bool KawasakiRS005LRobot::getSignal(int sigNo, bool &sigValue)
{
  str.clear();
  void *ptr = &sigNo;

  if (con->Send(packer->package(SIRRobotCommands::SIGNR,ptr)) != 0) {
    if (con->Receive(str, msWait) != 0) {
      if (parser->parse(str, &sigValue) == SIRRobotCommands::SIGNR)
        return true;
      else {
        (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getSignal: could not receive correct answer";
        return false;
      }
    }
    else {
      (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getSignal: could not receive answer";
      return false;
    }
  }
  else {
    (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::getSignal: could not send point";
    return false;
  }

}

bool KawasakiRS005LRobot::setSignal(int sigNo, bool state)
{
  str.clear();
  if(!state)
    sigNo=-sigNo;
  void *ptr = &sigNo;

  if (con->Send(packer->package(SIRRobotCommands::SIGNW,ptr)) != 0) {
    if (con->Receive(str, msWait) != 0) {
      if (parser->parse(str) == SIRRobotCommands::SIGNW)
        return true;
      else {
        (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::setSignal: could not receive correct answer";
        return false;
      }
    }
    else {
      (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::setSignal: could not receive answer";
      return false;
    }
  }
  else {
    (*logger) << SIRLOGTYPE::LOG_ERROR << "KawasakiRS005LRobot::setSignal: could not send point";
    return false;
  }

}
