#include "linatypes.h"


Odometry::Odometry()
{
	this->theta = 0;
	this->x = 0;
	this->y = 0;
}

LaserScan::LaserScan()
{
	this->size=0;
	this->data=NULL;
}

LaserScan::~LaserScan()
{
	delete[] this->data;
	this->data=NULL;
}



Odometry::Odometry(Odometry * o)
{
	if(o!=NULL)
	{
		this->theta = o->theta;
		this->x = o->x;
		this->y = o->y;
	}
}

LaserScan::LaserScan(LaserScan * ls)
{
	if(ls!=NULL)
	{
		this->size = ls->size;
		if(ls->data!=NULL && this->size>0)
		{
			this->data = new short[this->size];
			memcpy (this->data,ls->data,this->size*sizeof(short));
		}
		else
		{
			this->size=0;
			this->data = NULL;
		}
	}
	else {
		this->size=0;
		this->data=NULL;
	}
}

LaserScan * LaserScan::deserialize(char * buffer, int size)
{
	LaserScan * pRes = new LaserScan();
	if(pRes!=NULL && size>0 && size%2==0)
	{
		pRes->size=size/2;
		pRes->data=new short[pRes->size];
		for(int i=0; i<pRes->size; i++)
		{
			short d= StreamConverter::ToShort(buffer+i*2,2);
			pRes->data[i]=d;
		}	
	}
	else
		return NULL;

	return pRes;
	//if(size!=LASER_NB_MEASURES*2) // byte size = 2 * nb (sizeof(short)=2)
	//	return  NULL;

	//LaserScan * pRes = new LaserScan();
	//if(pRes!=NULL)
	//{
	//	pRes->size=LASER_NB_MEASURES;
	//	pRes->data=new short[LASER_NB_MEASURES];
	//	for(int i=0; i<LASER_NB_MEASURES; i++)
	//	{
	//		short d= StreamConverter::ToShort(buffer+i*2,2);
	//		pRes->data[i]=d;
	//	}	
	//}
	//else
	//	return NULL;

	//return pRes;
}

Odometry * Odometry::deserialize(char * buffer, int size)
{
	Odometry * pRes = new Odometry();
	pRes->x = StreamConverter::ToDouble(buffer,8);
	pRes->y = StreamConverter::ToDouble(buffer+8,8);
	pRes->theta = StreamConverter::ToDouble(buffer+16,8); 

	return pRes;
}

std::string LaserScan::toString()
{
	std::stringstream sres;
	sres << "Laser(";
	for(int i=0; i<this->size-1; i++) {
		sres << this->data[i] << ", ";
	}
	sres << this->data[this->size-1] << ")" << std::ends;
	return sres.str();
}

std::string Odometry::toString()
{
	std::stringstream sres;
	sres << "Odo(" << this->x << ", " << this->y << ", "<< this->theta << ")" << std::ends;
	return sres.str();
}
