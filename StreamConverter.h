#ifndef _STREAMCONVERTER_H_
#define _STREAMCONVERTER_H_

#include <iostream> 

class StreamConverter
{
public:
	static double ToDouble(char * buf, int size);
	static float ToFloat(char * buf, int size);
	static short ToShort(char * buf, int size);

private:
	StreamConverter();
};


#endif