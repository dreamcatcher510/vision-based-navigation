#ifndef _ROBOTLINK_H_
#define _ROBOTLINK_H_

#include "Poco/Thread.h"
#include "Poco/Mutex.h"

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <ios>

#include "linatypes.h"
#include "Client.h"
#include "HTTPCamera.h"
#include "OdometryThread.h"
#include "LaserThread.h"
#include "CameraThread.h"

class RobotLink
{
private:
	Poco::Thread leftCameraThread;
	Poco::Thread rightCameraThread;
	Poco::Thread laserThread;
	Poco::Thread odometryThread;
	
	Client  *m_driveClient;
	bool m_driveClientOK;

	Client *m_pantiltClient;
	bool m_pantiltClientOK;

	OdometryThread * m_odometryThread;
	Client * m_odometryClient;
	bool m_odometryClientOK;
	
	LaserThread * m_laserThread;
	Client * m_laserClient;
	bool m_laserClientOK;

	CameraThread * m_leftCameraThread;
	CameraThread * m_rightCameraThread;

	bool m_go_camera;
	bool m_go_laser;
	bool m_go_odometry;
	
	char * pBufferLaser;
	int bufferLaserSize;
	
	Odometry * pOdo;
	LaserScan * pLaser;
	Image * pImgL;
	Image * pImgR;
	

	Poco::Mutex mutex_Laser;
	Poco::Mutex mutex_Odo;
	Poco::Mutex mutex_ImgL;
	Poco::Mutex mutex_ImgR;

	char * m_leftCameraIPAddress;
	char * m_rightCameraIPAddress;
	char * m_robotIPAddress;
	int m_cameraPort;
	char * m_cameraContent;
public:
	RobotLink(char * robotIP);
	~RobotLink();
	Image * getLeftCameraFrame();
	Image * getRightCameraFrame();
	LaserScan * getLaserScan();
	Odometry * getOdometry();
	void setVlVa(float vl, float va, bool old);
	void setVlVa(double vl, double va);

	void setPanTilt(short pan, short tilt, short panSpeed, short tiltSpeed, char mode);
	
	void toogleLaser();
	void toogleOdometry();
	void toogleCamera();

	bool IsOdometryGoing() { return m_go_odometry; }
	bool IsLaserGoing() { return m_go_laser; }
	bool IsCameraGoing() { return m_go_camera; }

	char * getRobotIPAddress();
	void setLeftVideoString(char * ip, int port, char * content);
	void setRightVideoString(char * ip, int port, char * content);
	void getLeftVideoString(char ** ip, int * port, char ** content) { *ip=m_leftCameraIPAddress; *port=m_cameraPort; *content=m_cameraContent; }
	void getRightVideoString(char ** ip, int * port, char ** content) { *ip=m_rightCameraIPAddress; *port=m_cameraPort; *content=m_cameraContent; }

	void fillLeftImage(Image * img);
	void fillRightImage(Image * img);
	void fillOdometry(Odometry * odo);
	void fillLaser(LaserScan * odo);
};


#endif
