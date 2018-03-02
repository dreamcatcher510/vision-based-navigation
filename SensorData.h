#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include "Poco/Timestamp.h"

class SensorData
{
public:
	SensorData() { initializeTimestamp(); }
	unsigned long getTimestamp() { return m_timestamp; }
protected:
	unsigned long m_timestamp;
	void initializeTimestamp() {
		Poco::Timestamp now;
		this->m_timestamp = now.epochMicroseconds();
	}
};


#endif
