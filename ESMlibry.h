/*#============================================================================
//#
//#       Filename:  ESMlibry.h
//#
//#    Description:  Tracking structures and functions prototypes 
//#
//#
//#         Author:  Selim BENHIMANE & Ezio MALIS
//#        Company:  INRIA, Sophia-Antipolis, FRANCE
//#          Email:  Selim.Benhimane@in.tum.de
//#          Email:  Ezio.Malis@inria.fr
//#
//#
//#==========================================================================*/

#ifndef __ESMLIBRY_H
#define __ESMLIBRY_H



//#include <cv.h>
//#include <highgui.h>

#include "cv.h"
#include "highgui.h"

// #####   HEADER FILE INCLUDES   #############################################

// #####   MACROS  -  LOCAL TO THIS SOURCE FILE   #############################

// #####   EXPORTED MACROS   ##################################################

// #####   EXPORTED DATA TYPES   ##############################################

// #####   EXPORTED TYPE DEFINITIONS   ########################################

// Image Structure

#ifdef __cplusplus
	extern "C" {
#endif


typedef struct imageStruct {
  int    cols;  //!< width  / cols 
  int    rows;  //!< height / rows
  float *data;  //!< data
} imageStruct;

// Tracking Structure

typedef struct trackStruct {
  imageStruct** images; //!< various images  
  float**    trackdata; //!< various data
  int            miter; //!< iteration number 
  int            mprec; //!< tracking precision 
  float       homog[9]; //!< the homography computed 
} trackStruct;

// #####   EXPORTED VARIABLES   ###############################################

// #####   EXPORTED FUNCTION DECLARATIONS   ##################################

// Image Functions

int    MallImage    (imageStruct *image, int sizx, int sizy);
int    FreeImage    (imageStruct *image);
int    GetImageRows (imageStruct *image);
int    GetImageCols (imageStruct *image);
float* GetImageData (imageStruct *image);

// Input / Output Image Functions

int SavePgm (char *filename, imageStruct *image);
int ReadPgm (char *filename, imageStruct *image);

// Tracking Functions

int MallTrack     (trackStruct *trackArgs, imageStruct *image, int posx, int posy, int sizx, int sizy, int miter, int mprec);
int MakeTrack     (trackStruct *trackArgs, imageStruct *image);
int FreeTrack     (trackStruct *trackArgs);

int MallTrackMask (trackStruct *trackArgs, imageStruct *image, imageStruct *mask, int posx, int posy, int sizx, int sizy, int miter, int mprec);
int MakeTrackMask (trackStruct *trackArgs, imageStruct *image);
int FreeTrackMask (trackStruct *trackArgs);

imageStruct* GetPatm (trackStruct *trackArgs); //!< the mask pattern
imageStruct* GetPatr (trackStruct *trackArgs); //!< the reference pattern
imageStruct* GetPatc (trackStruct *trackArgs); //!< the current pattern
float        GetZNCC (trackStruct *trackArgs); //!< the zncc

#ifdef __cplusplus
	}
#endif

// #####   My FUNCTION DECLARATIONS   ##################################
void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2);
void DrawResult (int sx, int sy, float H[9], imageStruct I);
void DrawResultColor (int sx, int sy, float H[9], IplImage* I);
void DrawResultColor_Corners (cv::Point target_corners[], float H[9], IplImage* I);

void Homotransform (float H[9], std::vector<cv::Point2f> target_corners, std::vector<cv::Point2f> &scene_corners);
void plot_dot_mark( cv::Mat img, cv::Point pt, int radius, cv::Scalar color, int thickness, int line_type, int shift );
void PutLetterSubscript( cv::Mat img, std::string letter, std::string subscript, cv::Point pt, cv::Scalar color, double hscale, double vscale );
void template_mark( cv::Mat img, std::vector<cv::Point2f> obj_corners );

int IplImage2imageStruct ( IplImage* img, imageStruct **image);
int Mat2imageStruct (const cv::Mat &img, imageStruct **image);


#endif
