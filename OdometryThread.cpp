#include "OdometryThread.h"
#include "RobotLink.h"


OdometryThread::OdometryThread(RobotLink *rl) : SensorThread(rl)
{
	bufferOdometrySize = ODOMETRY_BUFFER_SIZE;
	pBufferOdometry = new char[bufferOdometrySize];
}


OdometryThread::~OdometryThread(void)
{
	if(pBufferOdometry!=NULL) { delete[] pBufferOdometry; pBufferOdometry = NULL; }
}

	
void OdometryThread::run()
{
	RobotLink * rl = this->m_robotLink;
	
	int nbTry = 0;
	int nbTryRcv = 0;
	while(true)
	{
		nbTryRcv = 0;
		std::cout << "Odometry begins !" << std::endl;
		while(rl->IsOdometryGoing()==false) { Poco::Thread::sleep(PARAMS_THREAD_WAIT_MS); }

		nbTry++;
		Client * odometryClient = new Client();
		bool odometryClientOK = odometryClient->init(rl->getRobotIPAddress(), NETWORK_ODOMETRY_PORT, NETWORK_UDP);

		while(odometryClientOK==true)
		{
			char * msg = new char[50];
			if(odometryClient->send(msg,50)==false)
			{
				odometryClientOK=false;
			}
			else
			{
				int size = odometryClient->receive(this->pBufferOdometry, this->bufferOdometrySize);
				if(size>0)
				{
					rl->fillOdometry(Odometry::deserialize(this->pBufferOdometry, size));
					//std::cout << rl->getOdometry()->toString() << std::endl;

					nbTryRcv = 0;
					nbTry=0;
				}
				else
				{
					nbTryRcv++;
					std::cout << "A problem occured when receiving Odometry." << std::endl;
				}
			}
			delete[] msg; msg = NULL;

			Poco::Thread::sleep(ODOMETRY_THREAD_WAIT);

			if(nbTryRcv>PARAMS_SOCKET_TRY_RECEIVE)
			{
				std::cout << "Too many problems when receiving Odometry." << std::endl;
				odometryClientOK=false;
			}
		}

		// Network link is probably lost
		// Free ressources and wait a little before trying again
		std::cout << "Odometry link is lost." << std::endl;
		delete odometryClient; odometryClient = NULL;
		Poco::Thread::sleep(PARAMS_SOCKET_TRY_WAIT);

		if(nbTry>PARAMS_SOCKET_TRY_NB)
		{
			std::cout << "Odometry link is definitively lost." << std::endl;
			return;
		}
	}
	
	return;
}