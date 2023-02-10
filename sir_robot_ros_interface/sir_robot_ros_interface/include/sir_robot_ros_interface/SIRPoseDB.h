#ifndef SIRPOSEDB_H
#define SIRPOSEDB_H
#include <Eigen/Dense>
#include "include/SIRRobot/SIRTypeDefs.h"

class SIRPoseDB
{
public:
	SIRPoseDB();
	~SIRPoseDB();
	virtual bool pushData(SIRMatrix &profile) = 0;
};

#endif

