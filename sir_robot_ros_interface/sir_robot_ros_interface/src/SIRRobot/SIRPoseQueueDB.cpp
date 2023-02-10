#include "SIRPoseQueueDB.h"
#include <iostream>


SIRPoseQueueDB::SIRPoseQueueDB()
{
}


SIRPoseQueueDB::~SIRPoseQueueDB()
{
}

bool SIRPoseQueueDB::pushData(SIRMatrix &pose)
{
	queue.push_back(new SIRMatrix(pose));
}

SIRMatrix* SIRPoseQueueDB::readData()
{
	return queue.front();
}

void SIRPoseQueueDB::popData()
{
	queue.pop_front();
}

int SIRPoseQueueDB::getDataNumber()
{
	return queue.size();
}

void SIRPoseQueueDB::printDB()
{
	std::cout << "--------- POSES -----------------" << std::endl;
	int j = 0;
	for (iter = queue.begin();iter != queue.end();iter++) {
		j++;
		std::cout << "POSE-" << j << " : ";
		std::cout << (*(*iter)) << std::endl;
		std::cout << std::endl;
	}
	std::cout << "------------------------------------" << std::endl;
}
