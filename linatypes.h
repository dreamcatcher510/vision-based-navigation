#ifndef _LINATYPES_H
#define _LINATYPES_H

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include "linaconstants.h"
#include "StreamConverter.h"


class LaserScan
{
public:
    int  size;
    short *data;
	
	LaserScan();
	~LaserScan();
	LaserScan(LaserScan * ls);
	static LaserScan * deserialize(char * buffer, int size);
	std::string toString();
};

class Odometry
{
public:
    double x;
	double y;
	double theta;

	Odometry();
	Odometry(Odometry* o);
	static Odometry * deserialize(char * buffer, int size);
	std::string toString();
};


#endif
