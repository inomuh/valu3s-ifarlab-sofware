#ifndef SIRTYPEDEFS_H
#define SIRTYPEDEFS_H

#include <Eigen/Core>

using Eigen::MatrixXd;
typedef Eigen::MatrixXd SIRMatrix;
#define PI 3.14159265
#define RAD PI/180
#define DEG 180/PI

/** An enum which defines the type of the points
It can be joint or task
*/
enum MOVEPOINTTYPE { //eklem ya da uc noktasý oldugunu belirlemek icin
	MPT_JOINT,
	MPT_TASK
};

/** An enum which defines the move types of the robot
It can be scan, P2P, linear, circular or none
*/
enum MOVETYPE { // hareket tipini belirlemek için
	MT_SCAN,
	MT_P2P,
	MT_LINEAR,
	MT_CIRCULAR,
	MT_NONE
};

/** An enum which defines the status of the robot
It can be stop, move or unknown
*/
enum ROBOTSTATUS { // robotun su anki durumunu belirlemek icin
	RS_STOP,
	RS_MOVE,
	RS_UNKNOWN
};





#endif