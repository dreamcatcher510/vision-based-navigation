#include "RobotLink.h"

RobotLink::RobotLink(char * robotIP)
{	
	m_go_camera = false;
	m_go_laser = false;
	m_go_odometry = false;
	
	pOdo = NULL;
	pLaser = NULL;
	pImgL = NULL;
	pImgR = NULL;

	m_leftCameraIPAddress=NULL;
	m_rightCameraIPAddress=NULL;
	m_robotIPAddress=NULL;
	m_cameraPort =-1;
	m_cameraContent = NULL;
	if(robotIP!=NULL)
	{
		m_robotIPAddress = robotIP;
	}
	else
	{
		m_robotIPAddress = NETWORK_ROBOT_IP;
	}



	// Manual mode : port 9003 UDP send byte : manual 1 / ...
	Client * modeClient = new Client();
	bool modeClientOK = modeClient->init(this->getRobotIPAddress(), NETWORK_SUPERVISOR_PORT, NETWORK_UDP);
	char * msg = new char[50]; msg[0] = 0x01;
	
	std::cout << "Setting manual mode... ";
	if(!modeClient->send(msg,50))
		std::cout << "Failed !!!" << std::endl;
	else
		std::cout << "OK !" << std::endl;
	
	// Client Vl Va Enable
	m_driveClient = new Client();
	m_driveClientOK = false;
	m_driveClientOK = m_driveClient->init(this->getRobotIPAddress(), NETWORK_DIFFERENTIAL_DRIVE_PORT, NETWORK_UDP);
	//m_driveClientOK = m_driveClient->init(this->getRobotIPAddress(), NETWORK_SPEEDS_PORT, NETWORK_UDP); // Kompai
	/*char * msg2 = new char[1024];
	m_driveClient->send(msg2, 1024);
	int t = m_driveClient->receive(msg2,300000); 
	std::cout << "Initialisation driveClient : " << t << std::endl;
	for(int z = 0; z<t; z++) std::cout << msg2[z];
	std::cout << std::endl;
	std::cout << "Fin initialisation driveClient" << std::endl;
	delete []msg2;*/

	m_pantiltClient = new Client();
	
	m_pantiltClientOK = false;
	m_pantiltClientOK = m_pantiltClient->init(this->getRobotIPAddress(), NETWORK_PAN_TILT_UNIT_PORT, NETWORK_UDP);
	

	m_odometryClient = new Client();
	m_odometryClientOK = false;
	m_odometryClientOK = m_odometryClient->init(this->getRobotIPAddress(), NETWORK_ODOMETRY_PORT, NETWORK_UDP);

	
	m_laserClient = new Client();
	m_laserClientOK = false;
	m_laserClientOK = m_laserClient->init(this->getRobotIPAddress(), NETWORK_LASER_PORT, NETWORK_UDP);


	m_odometryThread = new OdometryThread(this);
	m_laserThread = new LaserThread(this);
	m_leftCameraThread = new CameraThread(this, 'L');
	m_rightCameraThread = new CameraThread(this, 'R');
	leftCameraThread.start(*m_leftCameraThread);
	rightCameraThread.start(*m_rightCameraThread);
	laserThread.start(*m_laserThread);
	odometryThread.start(*m_odometryThread);
}

RobotLink::~RobotLink()
{
	if(pOdo!=NULL) { delete pOdo; pOdo=NULL; }
	if(pLaser!=NULL) { delete pLaser; pLaser=NULL; }
	if(pImgL!=NULL) { delete pImgL; pImgL=NULL; }
	if(pImgR!=NULL) { delete pImgR; pImgR=NULL; }
}

Image * RobotLink::getLeftCameraFrame()
{
	Image * res = NULL;

	mutex_ImgL.lock();
	{
		res = new Image(pImgL);
	}
	mutex_ImgL.unlock();

	return res;
}

Image * RobotLink::getRightCameraFrame()
{	
	Image * res = NULL;

	mutex_ImgR.lock();
	{
		res = new Image(pImgR);
	}
	mutex_ImgR.unlock();

	return res;
}

LaserScan * RobotLink::getLaserScan()
{
	LaserScan * res = NULL;
	
	mutex_Laser.lock();
	{
		res = new LaserScan(pLaser);
	}
	mutex_Laser.unlock();

	return res;
}

Odometry * RobotLink::getOdometry()
{
	Odometry * res = NULL;
	
	mutex_Odo.lock();
	{
		res = new Odometry(pOdo);
	}
	mutex_Odo.unlock();

	return res;
}

void RobotLink::setVlVa(double vl, double va)
{
	if(!m_driveClient)
	return;

	if(m_driveClientOK==true)
	{
		//char * msg = new char[50];
		char * msg = new char[16];

			int i=0;
			for(int k= 0; k<16; k++)
				msg[k]=0;
			*(double *)(msg+i) = vl; i+=8;
			*(double *)(msg+i) = va; i+=8;
			//*(bool *)(msg+i) = true; i+=1;
			/*for(int k= 0; k<50; k++)
			{
				std::cout << std::hex << std::setfill('0') << std::setw(2) << (unsigned short)msg[k];
			}
			std::cout << std::endl;*/
		
		if(!m_driveClient->send(msg,16))
		{
			std::cout << "Enabled to send Vl, Va." << std::endl;
		}
		delete[] msg; msg = NULL;
	}
	else
	{
		std::cout << "Enabled to send Vl, Va." << std::endl;
	}
}

void RobotLink::setPanTilt(short pan, short tilt, short panSpeed, short tiltSpeed, char mode)
{
	if(!m_pantiltClient)
		return;

	if(m_pantiltClientOK==true)
	{
		char * msg = new char[50];
		
		int i=0;
		*(char*)(msg+i) = mode; i+=1;		// DiDataPanTilt.Mode
		*(short*)(msg+i) = pan; i+=2;		// DiDataPanTilt.Pan
		*(short*)(msg+i) = tilt; i+=2;		// DiDataPanTilt.Tilt
		*(short*)(msg+i) = panSpeed; i+=2;	// DiDataPanTilt.PanSpeed
		*(short*)(msg+i) = tiltSpeed; i+=2;	// DiDataPanTilt.TiltSpeed
				
		if(!m_pantiltClient->send(msg,9))
		{
			std::cout << "Enabled to send Pan, Tilt." << std::endl;
		}
		delete[] msg; msg = NULL;
	}
	else
	{
		std::cout << "Enabled to send Pan, Tilt." << std::endl;
	}
}

void RobotLink::setVlVa(float vl, float va, bool old=false)
{
	if(!m_driveClient)
	{
		std::cout << "Client drive NOK" << std::endl;
		return;
	}

	if(m_driveClientOK==true)
	{
		char * msg = new char[50];

		if(old==true)
		{
			int i=0;
			*(msg+i) = vl; i+=4;
			*(msg+i) = va; i+=4;
			*(msg+i) = true; i+=1;
		}
		else
		{
			int i=0;
			*(float*)(msg+i) = vl; i+=4;	// Vl
			*(float*)(msg+i) = va; i+=4;	// Vab
			*(float*)(msg+i) = 0; i+=4;		// Vah
			*(float*)(msg+i) = 0; i+=4;		// VlShift
			*(float*)(msg+i) = 0; i+=4;		// RbShift
			*(float*)(msg+i) = 0; i+=4;		// RhShift
			*(float*)(msg+i) = 0; i+=4;		// VlMax
			*(float*)(msg+i) = 0; i+=4;		// VabMax
			*(float*)(msg+i) = 0; i+=4;		// VahMax
			*(bool*)(msg+i) = true; i+=1;	// Enable
			*(char*)(msg+i) = 0; i+=1;		// RobotType
			*(short*)(msg+i) = 0x000E; i+=2;// Masque (short)2|(short)4|(short)8|(short)256|(short)512; i+=2;
			*(char*)(msg+i) = 0; i+=1;		// ObstacleAvoidanceType
			*(short*)(msg+i) = 0; i+=2;		// nb Objectifs
			//*(msg+i) = ? ; i+= ? ;		// Liste Objectifs = {} si nb = 0
			*(short*)(msg+i) = 0; i+=2;		// Masque de RobotAdditionalInformation
		}
		if(!m_driveClient->send(msg,45))
		{
			std::cout << "Enabled to send Vl, Va." << std::endl;
		}

		delete[] msg; msg = NULL;
	}
	else
	{
		std::cout << "Enabled to send Vl, Va." << std::endl;
	}
}

void RobotLink::toogleLaser()
{
	if(this->m_go_laser==false)
		this->m_go_laser=true;
	else
		this->m_go_laser=false;
}

void RobotLink::toogleOdometry()
{
	if(this->m_go_odometry==false)
		this->m_go_odometry=true;
	else
		this->m_go_odometry=false;
}

void RobotLink::toogleCamera()
{
	if(this->m_go_camera==false)
		this->m_go_camera=true;
	else
		this->m_go_camera=false;
}


void RobotLink::fillLeftImage(Image * img)
{
	//Utilities::profile(std::string("RobotLink::fillImage"));
	mutex_ImgL.lock();
	{
		if(pImgL!=NULL) { delete pImgL; pImgL=NULL; }
		pImgL = img;
	}
	mutex_ImgL.unlock();
}

void RobotLink::fillRightImage(Image * img)
{ 
	//Utilities::profile(std::string("RobotLink::fillImage"));
	mutex_ImgR.lock();
	{
		if(pImgR!=NULL) { delete pImgR; pImgR=NULL; }
		pImgR = img;
	}
	mutex_ImgR.unlock();
}

void RobotLink::fillOdometry(Odometry * odo) 
{ 
	mutex_Odo.lock();
	{
		if(pOdo!=NULL) { delete pOdo; pOdo=NULL; }
		pOdo = odo;
	}
	mutex_Odo.unlock(); 
}

void RobotLink::fillLaser(LaserScan * laser) 
{ 
	mutex_Laser.lock(); 
	{
		if(pLaser!=NULL) { delete pLaser; pLaser=NULL; }
		pLaser = laser;
	}
	mutex_Laser.unlock(); 
}

void RobotLink::setLeftVideoString(char * ip, int port, char * content)
{
	m_leftCameraIPAddress = ip;
	m_cameraPort = port;
	m_cameraContent= content;
}

void RobotLink::setRightVideoString(char * ip, int port, char * content)
{
	m_rightCameraIPAddress = ip;
	m_cameraPort = port;
	m_cameraContent= content;
}

char * RobotLink::getRobotIPAddress()
{
	return m_robotIPAddress;
}
