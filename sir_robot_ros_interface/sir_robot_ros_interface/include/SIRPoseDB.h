#ifndef SIRPOSEDB_H
#define SIRPOSEDB_H
#include <Dense>
#include "SIRTypeDefs.h"

class SIRPoseDB
{
public:
	SIRPoseDB();
	~SIRPoseDB();
	virtual bool pushData(SIRMatrix &profile) = 0;
};

#endif

