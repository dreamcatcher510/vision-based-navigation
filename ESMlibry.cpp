/*
 * ESMlibry.cpp
 *
 *  Created on: Nov 19, 2012
 *      Author: robot
 */

#include "ESMlibry.h"
// #####   FUNCTION DEFINITIONS  -  LOCAL TO THIS SOURCE FILE   ###############

void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2)
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

void DrawResult (int sx, int sy, float H[9], imageStruct I)
{
  int i;
  float pEnd[8], pIni[8];

  pIni[0] =        0.0;    pIni[1] =          0.0;  pIni[2] = (float) sx-1;  pIni[3] =          0.0;
  pIni[4] = (float) sx-1;  pIni[5] = (float) sy-1;  pIni[6] =          0.0;  pIni[7] = (float) sy-1;
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

// draw a quadrangle vertices respect to the square tracking region
void DrawResultColor (int sx, int sy, float H[9], IplImage* I)
{
  int i;
  float pEnd[8], pIni[8];

  pIni[0] =        0.0;    pIni[1] =          0.0;  pIni[2] = (float) sx-1;  pIni[3] =          0.0;
  pIni[4] = (float) sx-1;  pIni[5] = (float) sy-1;  pIni[6] =          0.0;  pIni[7] = (float) sy-1;
  for (i = 0; i < 4; i++) {
    pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
    pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
  }
  cvDrawLine(I, cvPoint((int)pEnd[0],(int)pEnd[1]),cvPoint((int)pEnd[2],(int)pEnd[3]),cvScalar(0,0,255),1,8,0);
  cvDrawLine(I, cvPoint((int)pEnd[2],(int)pEnd[3]),cvPoint((int)pEnd[4],(int)pEnd[5]),cvScalar(0,0,255),1,8,0);
  cvDrawLine(I, cvPoint((int)pEnd[4],(int)pEnd[5]),cvPoint((int)pEnd[6],(int)pEnd[7]),cvScalar(0,0,255),1,8,0);
  cvDrawLine(I, cvPoint((int)pEnd[6],(int)pEnd[7]),cvPoint((int)pEnd[0],(int)pEnd[1]),cvScalar(0,0,255),1,8,0);

  return;
}

// transform the four target corners in the first frame to the current scene frame
void Homotransform (float H[9], std::vector<cv::Point2f> target_corners, std::vector<cv::Point2f> &scene_corners)
{
  int i;
  float pEnd[8], pIni[8];

  //// for 20120406
  cv::Point2f base = target_corners[0];
  // for 20120411
  //CvPoint base = target_corners[3];

  pIni[0] = (float)target_corners[0].x - base.x;  pIni[1] = (float)target_corners[0].y - base.y;
  pIni[2] = (float)target_corners[1].x - base.x;  pIni[3] = (float)target_corners[1].y - base.y;
  pIni[4] = (float)target_corners[2].x - base.x;  pIni[5] = (float)target_corners[2].y - base.y;
  pIni[6] = (float)target_corners[3].x - base.x;  pIni[7] = (float)target_corners[3].y - base.y;
//  pIni[0] = (float)target_corners[0].x;  pIni[1] = (float)target_corners[0].y;
//  pIni[2] = (float)target_corners[1].x;  pIni[3] = (float)target_corners[1].y;
//  pIni[4] = (float)target_corners[2].x;  pIni[5] = (float)target_corners[2].y;
//  pIni[6] = (float)target_corners[3].x;  pIni[7] = (float)target_corners[3].y;
  for (i = 0; i < 4; i++) {
    pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
    pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
  }
  scene_corners.clear();		// important !!!
  for(int i=0; i<4; i++)
  {
	  cv::Point2f pt(pEnd[2*i], pEnd[2*i+1]);
	  scene_corners.push_back(pt);
//	  scene_corners[i].x = pEnd[2*i];
//	  scene_corners[i].y = pEnd[2*i+1];
  }
  //cvDrawLine(I, cvPoint((int)pEnd[0],(int)pEnd[1]),cvPoint((int)pEnd[2],(int)pEnd[3]),cvScalar(0,255,0),1,8,0);
  //cvDrawLine(I, cvPoint((int)pEnd[2],(int)pEnd[3]),cvPoint((int)pEnd[4],(int)pEnd[5]),cvScalar(0,255,0),1,8,0);
  //cvDrawLine(I, cvPoint((int)pEnd[4],(int)pEnd[5]),cvPoint((int)pEnd[6],(int)pEnd[7]),cvScalar(0,255,0),1,8,0);
  //cvDrawLine(I, cvPoint((int)pEnd[6],(int)pEnd[7]),cvPoint((int)pEnd[0],(int)pEnd[1]),cvScalar(0,255,0),1,8,0);

  return;
}

// draw a quadrangle vertices respect to the four corners
void DrawResultColor_Corners (CvPoint target_corners[], float H[9], IplImage* I)
{
  int i;
  float pEnd[8], pIni[8];

  CvPoint base = target_corners[0];

  pIni[0] = (float)target_corners[0].x - base.x;  pIni[1] = (float)target_corners[0].y - base.y;
  pIni[2] = (float)target_corners[1].x - base.x;  pIni[3] = (float)target_corners[1].y - base.y;
  pIni[4] = (float)target_corners[2].x - base.x;  pIni[5] = (float)target_corners[2].y - base.y;
  pIni[6] = (float)target_corners[3].x - base.x;  pIni[7] = (float)target_corners[3].y - base.y;
  for (i = 0; i < 4; i++) {
    pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
    pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
  }
  cvDrawLine(I, cvPoint((int)pEnd[0],(int)pEnd[1]),cvPoint((int)pEnd[2],(int)pEnd[3]),cvScalar(0,255,0),1,8,0);
  cvDrawLine(I, cvPoint((int)pEnd[2],(int)pEnd[3]),cvPoint((int)pEnd[4],(int)pEnd[5]),cvScalar(0,255,0),1,8,0);
  cvDrawLine(I, cvPoint((int)pEnd[4],(int)pEnd[5]),cvPoint((int)pEnd[6],(int)pEnd[7]),cvScalar(0,255,0),1,8,0);
  cvDrawLine(I, cvPoint((int)pEnd[6],(int)pEnd[7]),cvPoint((int)pEnd[0],(int)pEnd[1]),cvScalar(0,255,0),1,8,0);

  return;
}


int IplImage2imageStruct ( IplImage* img, imageStruct **image)
{
	if( img->nChannels != 1 )
	{
		std::cout << "Please input gray image to the function IplImage2imageStruct() ... " << std::endl;
		return -1;
	}

	if( *image == NULL )
	{
		*image = new imageStruct;
		(*image)->rows = img->height;
		(*image)->cols = img->width;
		(*image)->data = new float[img->width*img->height];
	}
	else
	{
		delete[] (*image)->data;
		(*image)->data = NULL;
		delete *image;
		*image = NULL;

		*image = new imageStruct;
		(*image)->rows = img->height;
		(*image)->cols = img->width;
		(*image)->data = new float[img->width*img->height];
	}

	//image->rows = img->height;
	//image->cols = img->width;
	//image->clrs = img->nChannels;
	for(int i = 0; i< (*image)->rows; i++)
	{
		for(int j = 0; j< (*image)->cols; j++)
		{
			//*((*image)->data + i*((*image)->cols)+j*((*image)->clrs)) = (float)( img->imageData + img->widthStep*i)[j*img->nChannels];
			float temp = (float) (img->imageData[i*img->width+j]);
			*((*image)->data + i*((*image)->cols) + j) = temp >= 0 ? temp : temp + 255;
			//I_res_gray->imageData[i*I.cols+j] = (char)*(I.data + i*I.cols+j);
		}
	}
	return 0;
}

int Mat2imageStruct (const cv::Mat &img, imageStruct **image)
{
	if( img.channels() != 1 )
	{
		std::cout << "Please input gray image to the function Mat2imageStruct() ... " << std::endl;
		return -1;
	}

	if( *image == NULL )
	{
		*image = new imageStruct;
		(*image)->rows = img.rows;
		(*image)->cols = img.cols;
		(*image)->data = new float[(*image)->rows*(*image)->cols];
	}
	else
	{
		delete[] (*image)->data;
		(*image)->data = NULL;
		delete *image;
		*image = NULL;

		*image = new imageStruct;
		(*image)->rows = img.rows;
		(*image)->cols = img.cols;
		(*image)->data = new float[(*image)->rows*(*image)->cols];
	}

	for(int i = 0; i< (*image)->rows; i++)
	{
		for(int j = 0; j< (*image)->cols; j++)
		{
			*((*image)->data + i*((*image)->cols) + j) = (float)img.at<uchar>(i,j);
		}
	}

	return 0;
}

// my utils
void plot_dot_mark( cv::Mat img, cv::Point pt, int radius, cv::Scalar color, int thickness, int line_type, int shift )
{
	cv::circle( img, pt, radius, color, thickness, line_type, shift );
	int d = radius - 3 >= 0 ? radius - 3 : 0;
	cv::line( img, cv::Point(pt.x,pt.y-d), cv::Point(pt.x,pt.y+d), color, thickness, line_type, shift );
	cv::line( img, cv::Point(pt.x-d,pt.y), cv::Point(pt.x+d,pt.y), color, thickness, line_type, shift );
}

void PutLetterSubscript( cv::Mat img, std::string letter, std::string subscript, cv::Point pt, cv::Scalar color, double hscale, double vscale )
// pt the left buttom of the text
{
	cv::putText( img, letter, cvPoint(pt.x, pt.y), cv::FONT_HERSHEY_SIMPLEX, 1.0f, color, 1, 8, false);
	cv::putText( img, subscript, cvPoint(pt.x+15, pt.y), cv::FONT_HERSHEY_SIMPLEX, 1.0f, color, 1, 8, false);
}

void template_mark( cv::Mat img, std::vector<cv::Point2f> obj_corners )
{
	cv::line(img, cv::Point(obj_corners[0].x,obj_corners[0].y), cv::Point(obj_corners[1].x,obj_corners[1].y), cv::Scalar(0,255,0), 1, 8, 0 );
	cv::line(img, cv::Point(obj_corners[1].x,obj_corners[1].y), cv::Point(obj_corners[2].x,obj_corners[2].y), cv::Scalar(0,255,0), 1, 8, 0 );
	cv::line(img, cv::Point(obj_corners[2].x,obj_corners[2].y), cv::Point(obj_corners[3].x,obj_corners[3].y), cv::Scalar(0,255,0), 1, 8, 0 );
	cv::line(img, cv::Point(obj_corners[3].x,obj_corners[3].y), cv::Point(obj_corners[0].x,obj_corners[0].y), cv::Scalar(0,255,0), 1, 8, 0 );
	int radius = 10;
	plot_dot_mark( img, cv::Point(obj_corners[0].x,obj_corners[0].y), radius,  cv::Scalar(255,255,0), 1, 8, 0 );
	plot_dot_mark( img, cv::Point(obj_corners[1].x,obj_corners[1].y), radius,  cv::Scalar(0,0,255), 1, 8, 0 );
	plot_dot_mark( img, cv::Point(obj_corners[2].x,obj_corners[2].y), radius,  cv::Scalar(255,128,0), 1, 8, 0 );
	plot_dot_mark( img, cv::Point(obj_corners[3].x,obj_corners[3].y), radius,  cv::Scalar(255,0,255), 1, 8, 0 );

	/* initialize font and add text */
	PutLetterSubscript( img, "P", "0", cv::Point(obj_corners[0].x - 35, obj_corners[0].y - 5), cv::Scalar(255,255,0),  0.8, 0.8 );
	PutLetterSubscript( img, "P", "1", cv::Point(obj_corners[1].x + 10, obj_corners[1].y -5 ), cv::Scalar(0,0,255),  0.8, 0.8 );
	PutLetterSubscript( img, "P", "2", cv::Point(obj_corners[2].x + 10, obj_corners[2].y + 20), cv::Scalar(255,128,0),  0.8, 0.8 );
	PutLetterSubscript( img, "P", "3", cv::Point(obj_corners[3].x - 35, obj_corners[3].y + 20), cv::Scalar(255,0,255),  0.8, 0.8 );
}


