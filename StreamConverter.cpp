#include "StreamConverter.h"

double StreamConverter::ToDouble(char * buf, int size)
{
	double val = 0;
	if(size!=sizeof(double))
		std::cout << "To convert to double, buffer size has to be " << sizeof(double) << " bytes." << std::endl;
	else
	{
		double *pVal = (double *)buf;
		val = *pVal;
	}
	return val;
}

float StreamConverter::ToFloat(char * buf, int size)
{
	float val = 0;
	if(size!=sizeof(float))
		std::cout << "To convert to float, buffer size has to be "<< sizeof(float) << " bytes." << std::endl;
	else
	{
		float *pVal = (float *)buf;
		val = *pVal;
	}
	return val;
}

short StreamConverter::ToShort(char * buf, int size)
{
	short val = 0;
	if(size!=sizeof(short))
		std::cout << "To convert to short, buffer size has to be " << sizeof(short) << " bytes." << std::endl;
	else
	{
		short *pVal = (short *)buf;
		val = *pVal;
	}
	return val;
}

StreamConverter::StreamConverter() { }