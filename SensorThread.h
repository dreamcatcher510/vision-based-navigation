#ifndef _SENSORTHREAD_H_
#define _SENSORTHREAD_H_

#include "Poco/Runnable.h"

class RobotLink;

class SensorThread : public Poco::Runnable
{
private:
	SensorThread() {}
protected:
	RobotLink * m_robotLink;
public:
	virtual void run() = 0;
	SensorThread(RobotLink * rl) { m_robotLink = rl; }
	~SensorThread(void) { }

};

#endif //_SENSORTHREAD_H_
