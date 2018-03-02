#ifndef _ODOMETRY_THREAD_H_
#define _ODOMETRY_THREAD_H_


#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"

#include "SensorThread.h"


class RobotLink;

class OdometryThread : public SensorThread
{
private:
	char * pBufferOdometry;
	int bufferOdometrySize;

public:
	OdometryThread(RobotLink *rl);
	~OdometryThread();
	virtual void run();
};

#endif
