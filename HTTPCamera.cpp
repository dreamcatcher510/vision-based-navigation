#include "HTTPCamera.h"

HTTPCamera::HTTPCamera(RobotLink * rl, char* ip, int port, char *addr, char type)
{
	m_robotLink = rl;

	m_image_height = 480;
	m_image_width = 640;

	m_image = NULL;

	m_ipAddress = ip;
	m_port = port;
	m_request = addr;

	m_type = type;
}


HTTPCamera::~HTTPCamera()
{
}

void HTTPCamera::setImage(Image * img)
{
	m_image = img;
	if(m_robotLink!=NULL)
	{
		if(m_type=='L')
			m_robotLink->fillLeftImage(m_image);
		else if(m_type=='R')
			m_robotLink->fillRightImage(m_image);
	}
}

void HTTPCamera::openMJPEGStream()
{
	while(true)
	{
		while(m_ipAddress==NULL)
			Poco::Thread::sleep(500);

		// http://192.168.42.70/cgi-bin/encoder?USER=admin&PWD=123456&GET_STREAM
		Poco::Net::HTTPClientSession s(m_ipAddress, m_port); 
		Poco::Net::HTTPRequest request(Poco::Net::HTTPRequest::HTTP_GET, m_request);
		s.sendRequest(request);
		Poco::Net::HTTPResponse response;
		std::istream& rs = s.receiveResponse(response);

		MJPEGStreamProcessing * processing = new MJPEGStreamProcessing(this, &rs);
		Poco::Thread thread;
		thread.start(*processing);
		thread.join();
		std::cout << "MJPEGStreamProcessing has stopped" << std::endl;

		s.abort();

		delete processing;
		//Utilities::deletePtr((void**)&processing);
	}
}



