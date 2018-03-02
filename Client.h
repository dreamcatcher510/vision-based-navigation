#ifndef _CLIENT_H_
#define _CLIENT_H_


#include "Poco/Net/Socket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Net/StreamSocket.h"
#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketStream.h"
#include "Poco/Net/NetException.h"
#include "Poco/StreamCopier.h"
#include "Poco/Exception.h"

#include <iostream>
#include <sstream>
#include "linaconstants.h"

class Client
{
public:
	Client(void);
	~Client(void);
	bool init(char * ipAddress, short port, char type);
	bool send(char * msg, int length);
	int receive(char * msg, int length);
	void terminate();
	char getType() { return m_type; }
	Poco::Net::Socket * getSocket() { return m_socket; }
private:
	Poco::Net::SocketAddress * m_addrRobot;
	Poco::Net::SocketAddress * m_addrLocal;
	Poco::Net::Socket * m_socket;

	bool m_init;
	char m_type;
};

#endif

