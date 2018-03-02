#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <sstream>

#include "Poco/Timestamp.h"

#include "SensorData.h"

class Image : public SensorData
{
public:
	int width;
	int height;
	int nbChannels;

    char *data;

	Image();
	Image(Image* i);
	Image(char* buffer, int size); // Load from PPM buffer
	~Image();
	
	void initialize(int w, int h, int c);
	int getSize() { return size; }

private:
	int size; // Size of data in bytes
	void initialize();
};

#endif
