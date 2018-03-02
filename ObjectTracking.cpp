/*
 * ObjectTracking.cpp
 *
 *  Created on: Nov 17, 2012
 *      Author: robot
 */

#include "ObjectTracking.h"
#include "ESMlibry.h"


#define MAXFILENAME 100

int ObjectTracking::ESMTracking( cv::Mat )
{
	return 0;
}

int ObjectTracking::IplImageToimageStruct(const IplImage* img, imageStruct *image)
{
	if( img->nChannels != 1 )
	{
		std::cout << " the input image is not gray image" << std::endl;
		return -1;
	}
	image->rows = img->height;
	image->cols = img->width;
	for( int i = 0; i < image->rows; i++ )
	{
		for ( int j = 0; j < image->cols; j++ )
		{
			*( image->data + i*image->cols + j ) = (float) (img->imageData + img->widthStep*i)[j*img->nChannels];
		}
	}

	return 0;
}

void ObjectTracking::DrawLine(imageStruct *image, int r1, int c1, int r2, int c2)
{
  int dr, dc, temp;
  int cols = image->cols, rows = image->rows;
  int i, point, area;

  area = cols * rows;
  if (r1 == r2 && c1 == c2)
    return;
  if (abs (r2 - r1) < abs (c2 - c1)) {
    if (c1 > c2) {
      temp = r1; r1 = r2; r2 = temp;
      temp = c1; c1 = c2; c2 = temp;
    }
    dr = r2 - r1;
    dc = c2 - c1;
    temp = (r1 - c1 * dr / dc)*cols;
    for (i = c1; i <= c2; i++) {
      point = temp + (i * dr) / dc * cols  + i;
      if ((point >= 0) & (point < area))
	image->data[point] = 0.0;
    }
  }
  else {
    if (r1 > r2) {
      temp = r1; r1 = r2; r2 = temp;
      temp = c1; c1 = c2; c2 = temp;
    }
    dr = r2 - r1;
    dc = c2 - c1;
    temp =  c1 - r1 * dc / dr;
    for (i = r1; i <= r2; i++) {
      point = temp + i*cols + (i * dc) / dr;
      if ((point >= 0) & (point < area))
	image->data[point] = 0.0;
    }
  }

  return;
}

void ObjectTracking::DrawResult (int sx, int sy, float H[9], imageStruct I)
{
  int i;
  float pEnd[8], pIni[8];

  pIni[0] = 0;    pIni[1] = 0;      pIni[2] = sx - 1; pIni[3] = 0;
  pIni[4] = sx-1; pIni[5] = sy - 1; pIni[6] = 0;      pIni[7] = sy - 1;
  for (i = 0; i < 4; i++) {
    pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
    pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
  }
  DrawLine (&I, (int)pEnd[1], (int)pEnd[0], (int)pEnd[3], (int)pEnd[2]);
  DrawLine (&I, (int)pEnd[3], (int)pEnd[2], (int)pEnd[5], (int)pEnd[4]);
  DrawLine (&I, (int)pEnd[5], (int)pEnd[4], (int)pEnd[7], (int)pEnd[6]);
  DrawLine (&I, (int)pEnd[7], (int)pEnd[6], (int)pEnd[1], (int)pEnd[0]);

  return;
}
