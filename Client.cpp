#include "Client.h"


Client::Client()
{
	m_init = false;
	m_type = '\0';
}


Client::~Client()
{
	this->terminate();
}

bool Client::init(char * ipAddress, short port, char type)
{
	std::cout << "Initialization of socket to " << ipAddress << ":" << port << std::endl;

	m_type = type;

	std::ostringstream oss;
	oss << ipAddress << ':' << port;

	m_addrRobot = new Poco::Net::SocketAddress(oss.str());
	m_addrLocal = new Poco::Net::SocketAddress("localhost", port);

	try
	{
		switch(m_type)
		{
			case NETWORK_TCP:
				m_socket= new Poco::Net::StreamSocket(*m_addrRobot);
			
				m_init = true;
				return true;
				break;

			case NETWORK_UDP:
				m_socket = new Poco::Net::DatagramSocket();
				((Poco::Net::DatagramSocket *)m_socket)->connect(*m_addrRobot);

				m_init = true;
				return true;
				break;

			default:
				m_init = false;
				return false;
		}
	}
	catch(const Poco::Net::NetException & e)
	{
		std::cout << e.displayText() << std::endl;
		terminate();
	}
	catch(const Poco::IOException & e)
	{
		std::cout << e.displayText() << std::endl;
		terminate();
	}
	catch(...)
	{
		std::cout << "Exception inconnue" << std::endl;
		terminate();
	}
}

bool Client::send(char * msg, int length)
{
	switch(m_type)
	{
	case NETWORK_TCP:
		((Poco::Net::StreamSocket *)m_socket)->sendBytes(msg, length);

		return true;
		break;
	case NETWORK_UDP:
		if(m_socket!=NULL)
			((Poco::Net::DatagramSocket *)m_socket)->sendBytes(msg, length);

		return true;
		break;
	default:
		return false;
	}
}

int Client::receive(char * msg, int length)
{	
	int size = -1;

	switch(m_type)
	{
	case NETWORK_TCP:
	{
		size = ((Poco::Net::StreamSocket *)m_socket)->receiveBytes(msg, length);
		break;
	}
	case NETWORK_UDP:
	{
		Poco::Net::SocketAddress sender;
		size = ((Poco::Net::DatagramSocket *)m_socket)->receiveFrom(msg, length, sender);
		break;
	}
	default:
	{
		return -1;
	}
	}
	
	if(size<=0)
	{
		std::cout << "Error during receive (no data read)" << std::endl;
	}
	return size;
}

void Client::terminate()
{
	if(m_socket!=NULL) {
		m_socket->close();
		delete m_socket;
	}

	if(m_addrRobot!=NULL)
		delete m_addrRobot;
	if(m_addrLocal!=NULL)
		delete m_addrLocal;

	m_init=false;
}