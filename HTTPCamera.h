#ifndef _HTTP_CAMERA_H_
#define _HTTP_CAMERA_H_

#include <cstdlib>
#include <iostream>
#include <string>
#include "Poco/Net/HTTPClientSession.h"
#include "Poco/Net/HTTPRequest.h"
#include "Poco/Net/HTMLForm.h"
#include "Poco/Net/HTTPResponse.h"
#include "Poco/Thread.h"
#include "Poco/RunnableAdapter.h"
#include "Poco/FileStream.h"
#include "Poco/DateTime.h"
#include "Poco/DateTimeFormat.h"
#include "Poco/DateTimeFormatter.h"
#include "Poco/BinaryWriter.h"
#include "Poco/String.h"
#include "Poco/StringTokenizer.h"
#include "Poco/RegularExpression.h"
#include "Poco/NumberParser.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "linatypes.h"
#include "Image.h"
#include "RobotLink.h"
#include "Utilities.h"


class RobotLink;

class HTTPCamera
{
public:
	HTTPCamera(RobotLink* rl, char * ip, int port, char * content, char type);
	~HTTPCamera();

	void openMJPEGStream();
	int getWidth() { return m_image_width; }
	int getHeight() { return m_image_width; }
	void setImage(Image * img);

private:
	int m_image_height;
	int m_image_width;
	
	char * m_ipAddress;
	int m_port;
	char * m_request;

	Image * m_image;

	RobotLink * m_robotLink;

	char m_type;

};






class MJPEGStreamProcessing: public Poco::Runnable
{
private:
	char * m_buffer;
	int m_buffer_size;
	HTTPCamera *m_cam;
	std::istream * m_rs;

public:
	MJPEGStreamProcessing(HTTPCamera * cam, std::istream * rs)
	{
		m_buffer_size = 150000; // ~48 ko en 100%
		m_buffer = new char[m_buffer_size];
		m_cam = cam;
		m_rs = rs;
	}

	~MJPEGStreamProcessing() {
		Utilities::deletePtrTab((void**)&m_buffer);
	} 

	void run()
	{
		int countKo=0,tailleRestant=0;
		char * restant = new char[m_buffer_size];
		while(true)
		{
			//Utilities::profile(std::string("Image"));

			if(m_buffer==NULL) {

				m_buffer = new char[m_buffer_size];
				if(m_buffer==NULL)
					std::cout << "Buffer uninitialized... Not enough memory" << std::endl;
				continue;
			}
			
			for(int i=0; i<tailleRestant; i++)
			{
				m_buffer[i]=restant[i];
			}

			m_rs->read(m_buffer+tailleRestant, m_buffer_size-tailleRestant);
			

			tailleRestant = 0;

			int posStart = -1;
			for(int  i=0; i<m_buffer_size-2; i++)
			{
				if(m_buffer[i]=='-' && m_buffer[i+1]=='-' && m_buffer[i+2]=='G')
				{
					posStart=i;
					break;
				}
				
			}


			if(posStart>=0)
			{
				int lengthDeb = Utilities::find(m_buffer+posStart, m_buffer_size-posStart, "Length: ", 4);
				lengthDeb += posStart+8; // length("Length: ")=8
				char * lengthEndPtr;
				long int jpegSize = strtol(m_buffer+lengthDeb, &lengthEndPtr, 10);
			
				int jpegDeb = lengthEndPtr - m_buffer + 4;
				if(jpegDeb+jpegSize<m_buffer_size)
				{
					Image * img;
					//Utilities::profile("JPEG");
					Utilities::jpegToImage(m_buffer+jpegDeb, jpegSize, &img);
					//Utilities::profile("JPEG");
					m_cam->setImage(img);
					countKo = 0;

				}
				else
				{
					std::cout << "Buffer size for images is too short." << std::endl;
				}
				
				if(m_buffer!=NULL)
				{
					for(int i=jpegSize+jpegDeb; i<m_buffer_size;i++)
					{
						restant[tailleRestant]=m_buffer[i];
						tailleRestant++;
					}
				} 
				else
				{
					tailleRestant = 0;
				}

			}
			else
			{
				std::cout << "Boundary has not been found." << std::endl;
				countKo++;
				if(countKo>= CAMERA_NB_FAILS)
				{
					std::cout << "Camera link lost, going to retry..." << std::endl;
				}
			}

		}

		if(restant!=NULL)
		{
			delete [] restant;
			restant = NULL;
			tailleRestant = 0;
		}

	}
};



#endif
