#include "Image.h"


Image::Image()
{
	this->initialize();
}

Image::Image(Image* i)
{
	if(i==NULL)
	{
		this->initialize();
		return;
	}
	
	if(i->data==NULL || i->height<1 || i->width<1 || i->nbChannels<1)
	{		
		this->initialize();
		return;
	}

	this->initialize(i->width, i->height, i->nbChannels);
	std::memcpy(this->data, i->data, i->size); // cpy data (i->data);
}


Image::Image(char* buffer, int size)
{
	if(size<10 || buffer==NULL) {
		this->initialize();
		return;
	}
	
	int ptrStart = 0; int ptrEnd = 0;
	while(buffer[ptrEnd]!=0x0a && ptrEnd<size) { ptrEnd++; } // P6 0x0a
	if(ptrEnd>=size) {
		this->initialize();
		return;
	}

	ptrStart = ptrEnd+1; ptrEnd++;
	this->width = atoi(buffer+ptrStart);
	while(buffer[ptrEnd]!=0x20 && ptrEnd<size) { ptrEnd++; } // P6 0x0a Width 0x20
	if(ptrEnd>=size) {
		this->initialize();
		return;
	}

	ptrStart = ptrEnd+1; ptrEnd++;
	this->height = atoi(buffer+ptrStart);
	while(buffer[ptrEnd]!=0x0a && ptrEnd<size) { ptrEnd++; } // P6 0x0a Width 0x20 Height 0x0a
	if(ptrEnd>=size) {
		this->initialize();
		return;
	}
	
	ptrStart = ptrEnd+1; ptrEnd++;
	int maxValChan = atoi(buffer+ptrStart);
	while(buffer[ptrEnd]!=0x0a && ptrEnd<size) { ptrEnd++; } // P6 0x0a Width 0x20 Height 0x0a MaxValChannel 0x0a
	if(ptrEnd>=size) {
		this->initialize();
		return;
	}
	
	ptrStart = ptrEnd+1; ptrEnd++;
	
	this->nbChannels = 3;
	this->size = this->width*this->height*this->nbChannels;
	this->data = new char[this->size];
	for(int i=0; i<this->size; i++)
	{
		this->data[i]=(buffer+ptrStart)[i];
	}
}

void Image::initialize()
{
	this->data=NULL;
	this->size=-1;
	this->width=-1;
	this->height=-1;
}

void Image::initialize(int w, int h, int c)
{
	if(w>0 && h>0 && c>0)
	{
		this->width=w;
		this->height=h;
		this->nbChannels=c;
		this->size = this->width*this->height*this->nbChannels;
		this->data = new char[this->size];
	}
	else
	{
		initialize();
	}
}

Image::~Image()
{
	if(this->data!=NULL)
		delete [] this->data;
}
