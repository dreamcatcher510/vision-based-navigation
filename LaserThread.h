#ifndef _LASER_THREAD_H_
#define _LASER_THREAD_H_

#include "SensorThread.h"

class RobotLink;

class LaserThread : public SensorThread
{
private:
	char * pBufferLaser;
	int bufferLaserSize;

public:
	LaserThread(RobotLink *rl);
	~LaserThread();
	virtual void run();
};


#endif

