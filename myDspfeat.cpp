/*
This program detects image features using SIFT keypoints. For more info,
refer to:

Lowe, D. Distinctive image features from scale-invariant keypoints.
International Journal of Computer Vision, 60, 2 (2004), pp.91--110.

Copyright (C) 2006-2010  Rob Hess <hess@eecs.oregonstate.edu>

Note: The SIFT algorithm is patented in the United States and cannot be
used in commercial products without a license from the University of
British Columbia.  For more information, refer to the file LICENSE.ubc
that accompanied this distribution.

Version: 1.1.2-20100521
*/

#include <highgui.h>

#include <stdio.h>
#include <iostream>
#include "myDspfeat.h"

int myDspFeat( IplImage* img, struct feature* features, int n, char* win_name, char* out_img_name )
{
	if( ! img )
	{
		std::cout << "image is not avaliable for displacement..." <<std::endl;
		return -1;
	}
	IplImage* imgdsp = cvCreateImage(cvGetSize(img), img->depth,img->nChannels);
	cvCopy(img, imgdsp, NULL);

	draw_features( imgdsp, features, n );
	cvNamedWindow( win_name, 1 );
	cvShowImage( win_name, imgdsp );
	//cvWaitKey( 0 );

	if( out_img_name != NULL )
		cvSaveImage( out_img_name, imgdsp );

	cvDestroyWindow(win_name);
	if(img->imageDataOrigin) cvReleaseImage( &imgdsp );

	std::cout << "displace image feature" << win_name << std::endl;
	return 0;
}
