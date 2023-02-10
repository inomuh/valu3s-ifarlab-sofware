#ifndef SIRPOSEQUEUEDB_H
#define SIRPOSEQUEUEDB_H

#include "include/SIRRobot/SIRPoseDB.h"
#include <list>
class SIRPoseQueueDB : 	public SIRPoseDB{
public:
	SIRPoseQueueDB();
	~SIRPoseQueueDB();
	//push the data to the end of queue
	bool pushData(SIRMatrix &pose);
	//pop the data from the begin of queue
	void popData();
	//read the data from the begin of queue
	SIRMatrix* readData();
	//return the number of profiles in the queue
	int getDataNumber();

	//print database
	void printDB();
private:
	std::list<SIRMatrix *> queue;
	std::list<SIRMatrix *>::iterator iter;
};

#endif

