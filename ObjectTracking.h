/*
 * ObjectTracking.h
 *
 *  Created on: Nov 17, 2012
 *      Author: robot
 */

#ifndef OBJECTTRACKING_H_
#define OBJECTTRACKING_H_

#include <opencv2/opencv.hpp>
#include "ESMlibry.h"

class ObjectTracking{
public:
	/*
	 *  ESM tracking
	 */
	int ESMTracking(cv::Mat);

	/*
	 *  image conver
	 */
	int IplImageToimageStruct(const IplImage* img, imageStruct *image);

	/*
	 *  ESM draw function
	 */
	void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2);
	void DrawResult (int sx, int sy, float H[9], imageStruct I);
};


#endif /* OBJECTTRACKING_H_ */
