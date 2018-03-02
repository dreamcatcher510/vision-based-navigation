#ifndef _CAMERATHREAD_H_
#define _CAMERATHREAD_H_

#include "SensorThread.h"

class RobotLink;

class CameraThread : public SensorThread
{
private:
	char m_type;
public:
	CameraThread(RobotLink * rl, char type);
	~CameraThread(void);
	virtual void run();
};

#endif //_CAMERATHREAD_H_
