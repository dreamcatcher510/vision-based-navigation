#include "LaserThread.h"
#include "RobotLink.h"


LaserThread::LaserThread(RobotLink* rl) : SensorThread(rl)
{
	bufferLaserSize = LASER_BUFFER_SIZE;
	pBufferLaser = new char[bufferLaserSize];
}


LaserThread::~LaserThread(void)
{
	if(pBufferLaser!=NULL) { delete[] pBufferLaser; pBufferLaser = NULL; }
}
	
void LaserThread::run()
{
	RobotLink * rl = m_robotLink;
	
	int nbTry = 0;
	int nbTryRcv = 0;
	while(true)
	{
		std::cout << "Laser begins !" << std::endl;
		while(rl->IsLaserGoing()==false) { Poco::Thread::sleep(PARAMS_THREAD_WAIT_MS); }

		nbTry++;
		Client * laserClient = new Client();
		bool laserClientOK = laserClient->init(rl->getRobotIPAddress(), NETWORK_LASER_PORT, NETWORK_UDP);

		while(laserClientOK==true)
		{
			char * msg = new char[50];
			if(laserClient->send(msg,50)==false)
			{
				laserClientOK=false;
			}
			else
			{
				int size = laserClient->receive(pBufferLaser, bufferLaserSize);
				if(size>0)
				{
					LaserScan * s = LaserScan::deserialize(pBufferLaser, size);
					if(s!=NULL && s->data!=NULL)
					{
						rl->fillLaser(s);
						//std::cout << rl->getLaserScan()->toString() << std::endl;

						nbTryRcv = 0;
						nbTry=0;

					}
					else
					{
						nbTryRcv++;
						std::cout << "A problem occured when deserializing laser." << std::endl;
					}
				}
				else
				{
					nbTryRcv++;
					std::cout << "A problem occured when receiving Laser." << std::endl;
				}
			}
			delete[] msg;
			msg=NULL;

			Poco::Thread::sleep(LASER_THREAD_WAIT);

			if(nbTryRcv>PARAMS_SOCKET_TRY_RECEIVE)
			{
				std::cout << "Too many problems when receiving Laser." << std::endl;
				laserClientOK=false;
			}
		}

		// Network link is probably lost
		// Free ressources and wait a little before trying again
		std::cout << "Laser link is lost." << std::endl;
		delete laserClient;
		laserClient = NULL;
		Poco::Thread::sleep(PARAMS_SOCKET_TRY_WAIT);

		if(nbTry>PARAMS_SOCKET_TRY_NB)
		{
			std::cout << "Laser link is definitively lost." << std::endl;
			return;
		}
	}

	return;
}
