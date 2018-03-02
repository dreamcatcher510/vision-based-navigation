#include "CameraThread.h"
#include "RobotLink.h"


CameraThread::CameraThread(RobotLink * rl, char type) : SensorThread(rl)
{
	m_type = type;
}


CameraThread::~CameraThread(void)
{
}

void CameraThread::run()
{
	RobotLink * rl = m_robotLink;

	char * addr=NULL;
	int port=-1;
	char * content=NULL;
	do
	{
		if(m_type=='L')
			rl->getLeftVideoString(&addr, &port, &content);
		else if(m_type=='R')
			rl->getRightVideoString(&addr, &port, &content);
		else
		{
			Poco::Thread::sleep(100);
			continue;
		}
		Poco::Thread::sleep(100);
	} while(addr==NULL);
	
	HTTPCamera * cam = new HTTPCamera(rl, addr, port, content, m_type);
	cam->openMJPEGStream();

	while(true) { Poco::Thread::sleep(5000); }
	/*while(rl->m_go_camera==false) { psleep(PARAMS_THREAD_WAIT_MS); }

	while(true)
	{
		IplImage * img = cam->getFrame();
		Image * image = new Image();
		image->data = img;
		rl->fillImage(image);
	}*/

	/*CvCapture *camera=cvCaptureFromFile(rl->m_videoString); 
    if (camera==NULL) 
        printf("camera is null\n"); 
    else 
	{
        printf("camera is not null"); 
 
		cvNamedWindow("img"); 
		while (true){ 
			try{
				double t1=(double)cvGetTickCount(); 
				IplImage *img=cvQueryFrame(camera); 
				double t2=(double)cvGetTickCount(); 
				printf("time: %gms  fps: %.2g\n",(t2-t1)/(cvGetTickFrequency()*1000.), 1000./((t2-t1)/(cvGetTickFrequency()*1000.))); 
				if(img!=NULL)
					cvShowImage("img",img); 
				else
					std::cout << "Error on retreived image" << std::endl;
			} catch( ... ) {}
			psleep(100);
		} 
	}
	cvReleaseCapture(&camera); */


	return;
}
