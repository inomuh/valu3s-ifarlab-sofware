#ifndef SIRPOSEDB_H
#define SIRPOSEDB_H
#include <eigen3/Eigen/Dense>
#include "SIRTypeDefs.h"

class SIRPoseDB
{
public:
	SIRPoseDB();
	~SIRPoseDB();
	virtual bool pushData(SIRMatrix &profile) = 0;
};

#endif

