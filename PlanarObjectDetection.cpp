/*
 * PlanarObjectDetection.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: robot
 */


/*
 * PlanarObjectDetection.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: robot
 */

#include <fstream>
#include "PlanarObjectDetection.h"

extern "C" {
#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"
}
#include "myDspfeat.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpMomentObject.h>
#include <visp/vpPoint.h>
#include <visp/vpMomentGravityCenter.h>

#include "myUtilities.h"

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/imgproc/imgproc_c.h>
//#include <opencv2/opencv.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/opencv_modules.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/flann/flann.hpp>
//#include <opencv2/legacy/legacy.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

std::string fTargetCorners  = "../configurations/target_cornerpoints_I1_000001.txt";
std::string fTargetLbgSquareLUpt  = "../configurations/target square_I1_000001.txt";

PlanarObjectDetection::PlanarObjectDetection()
{}

int PlanarObjectDetection::objectDetection(const cv::Mat &I_object,
											const cv::Mat &I_scene,
											const cv::Mat cameraMatrix)

{
	bool success = true;
	/*
	 *  -- step 1: prepoccessing
	 */

	/*
	 * 	check the image data and convert image structure
	 */
	if( I_object.empty() || I_scene.empty())
	{
		std::cout << "error with the images of matching ... " << std::endl;
		return -1;
	}



//	cv::namedWindow("SIFT Matches & Object detection");
//	cv::Mat img_matches;
//	cv::drawMatches( I_object, std::vector<cv::KeyPoint> (), I_scene, std::vector<cv::KeyPoint> (),
//						std::vector<cv::DMatch> (), img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
//						std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//	cv::imshow("SIFT Matches & Object detection", img_matches);
//	cv::waitKey(0);
//	cv::destroyWindow( "SIFT Matches & Object detection" );


	/*
	 * 	read the object corners (quadrilateral) in the projection image of object
	 */
//	CvPoint obj_corners[4];		// the corner of object
//	std::ifstream ifTargetCorners(fTargetCorners);
//	if (ifTargetCorners.fail())
//	{
//		std::string error = "Unable to open file " + fTargetCorners;
//		throw std::invalid_argument(error);
//	}
//	for(int i=0; i<8; i++)
//	{
//		if(i<4)
//		{
//			ifTargetCorners >> obj_corners[0].x;
//			ifTargetCorners >> obj_corners[0].y;
//		}
//		else
//		{
//			ifTargetCorners >> obj_corners[i-4].x;
//			ifTargetCorners >> obj_corners[i-4].y;
//		}
//	}
//	std::cout<< "load target template coordinates from file... " << std::endl;

	/*
	 *	read the leftup corner coordinate of the target region in the reference image
	 */
//	std::ifstream ifTargetLbgSquareLUpt( fTargetLbgSquareLUpt );
//	if (ifTargetLbgSquareLUpt.fail())
//	{
//		std::string error = "Unable to open file " + fTargetLbgSquareLUpt;
//		throw std::invalid_argument(error);
//	}
//	int x_m, y_m, width_m, height_m;
//
//	#if defined NO_CROPPING
//		x_m = 0;
//		y_m = 0;
//		width_m = target->width;
//		height_m = target->height;
//	#elif defined NOBKGD
//		for(int i=0; i<4; i++)
//			ifTargetLbgSquareLUpt >> x_m;
//		ifTargetLbgSquareLUpt >> x_m;
//		ifTargetLbgSquareLUpt >> y_m;
//		ifTargetLbgSquareLUpt >> width_m;
//		ifTargetLbgSquareLUpt >> height_m;
//	#elif defined LILBKGD
//		ifTargetLbgSquareLUpt >> x_m;
//		ifTargetLbgSquareLUpt >> y_m;
//		ifTargetLbgSquareLUpt >> width_m;
//		ifTargetLbgSquareLUpt >> height_m;
//	#endif
//		std::cout<< "load the coordinate of point Pm from file... " << std::endl;

	/*
	 *  -- step 2: Detect the keypoints using SIFT detector
	 *  		   and calculate descriptors (feature vectors)
	 */
	int n1, n2;
	struct feature* feat1, * feat2;
	IplImage Ipl_1 = I_object.operator _IplImage();
	IplImage Ipl_2 = I_scene.operator _IplImage();

	IplImage* img1 = &Ipl_1;
	IplImage* img2 = &Ipl_2;

	n1 = sift_features( img1, &feat1 );
	std::cout << "Finding %d features in the projection image of object ..." << n1 << std::endl;
	n2 = sift_features( img2, &feat2 );
	std::cout << "Finding %d features in the projection image of scene ..." << n2 << std::endl;

	//--------------------- displace features ---------------------//
	myDspFeat( img1,  feat1, n1, "reference image features", NULL );
	myDspFeat( img2,  feat2, n2, "scene image features", NULL );

	/*
	 *  -- step 3: matching descriptor vectors using kd-tree
	 */
	std::cout << "Building kd tree ..." << std::endl;
	struct kd_node* kd_root;
	kd_root = kdtree_build( feat2, n2 );
	struct feature* feat;
	struct feature** nbrs;
	int k, m = 0;
	double d0, d1;
	CvPoint pt1, pt2;
	for( int i = 0; i < n1; i++ )
	{
		feat = feat1 + i;
		k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
		if( k == 2 )
		{
			d0 = descr_dist_sq( feat, nbrs[0] );
			d1 = descr_dist_sq( feat, nbrs[1] );
			if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
			{
				pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
				pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
				pt2.y += img1->height;
				// cvLine( stacked, pt1, pt2, CV_RGB(255,0,255), 1, 8, 0 );
				m++;
				feat1[i].fwd_match = nbrs[0];
			}
		}
		free( nbrs );
	}
	std::cout << "Found total matches : " << m << std::endl;

	/*
	 *  -- step 4: (optional) get "good" matches
	 */

	/*
	 *  -- step 5: calculate the homography transformation
	 */
	std::vector<cv::Point2f> points_object, points_scene;
	for ( int i = 0; i < n1; i++ )
	{
		if ( feat1[i].fwd_match )
		{
			points_object.push_back( cv::Point2f(feat1[i].x, feat1[i].y) );
			points_scene.push_back( cv::Point2f(feat1[i].fwd_match->x, feat1[i].fwd_match->y) );
		}
	}
	std::vector<uchar> inliers;
	//cv::Mat H;
	if ( points_object.size() >= 4 )
	{
		H_pixel = cv::findHomography(points_object, points_scene, inliers, cv::RANSAC, 1);
		// H = cv::findHomography(points_object, points_scene, inliers, cv::RANSAC, 1);
		H = cameraMatrix.inv()*H_pixel*cameraMatrix;	// Hm = C.inv()*Hp*C;

	}
	else
	{
		std::cout << "there are not enough points to calculate homography transformation ..." << std::endl;
		return -1;
	}

	/*
	 *  -- step 6: verify the detection result using Homography decomposition
	 */
	if ( planarMotionConstraintVerification(H) != 0 )
	{
		std::cout << "homography dose not content the planar motion constraint ..." << std::endl;
		success = false;
	}

	/*
	 *  -- step 7: localize the object in the projection of scene image by homography transformation
	 */


	// set the corners in the projection image of the scene ( locate/identify the object to be "detected" )
	std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cv::Point2f(168,66); obj_corners[1] = cv::Point2f( 480, 61 );			// the corner of object/reference can be get from file
	obj_corners[2] = cv::Point2f( 477, 467 ); obj_corners[3] = cv::Point2f( 174, 466 );
	object_corners.clear();			// important !!!
	object_corners.push_back(obj_corners[0]);
	object_corners.push_back(obj_corners[1]);
	object_corners.push_back(obj_corners[2]);
	object_corners.push_back(obj_corners[3]);
//	// calculate the object points in meter
//	std::vector<cv::Point3f> obj_corners_homo;
//	obj_corners_homo.reserve(4);
//	for(int i = 0; i < object_corners.size(); i++)
//	{
//		obj_corners_homo.push_back( cv::Point3f(obj_corners[i].x, obj_corners[i].y, 1.0f) );
//	}
//	// pixel meter conversion using camera model
//	cv::Mat object_corner_Mat = (cv::Mat_<double>(3,4) << 168, 480, 477, 174, 66, 61, 467, 466, 1, 1, 1, 1);
//	cv::Mat object_cornerinmeter_Mat = cameraMatrix.inv()*object_corner_Mat;
//	object_corners_in_meter.clear();			// important !!!
//	for(int i = 0; i < object_corners.size(); i++ )
//	{
//		cv::Vec3f vec = object_cornerinmeter_Mat.col(i);
//		cv::Point3f pt(vec[0], vec[1], vec[2]);
//		object_corners_in_meter.push_back(pt);
//	}

	// converse the object corners from pixel to meter
	pixeltometer(cameraMatrix,	object_corners, object_corners_meter);

	if ( !H_pixel.empty() )
		cv::perspectiveTransform( obj_corners, scene_corners_initial, H_pixel);
	for( int i=0; i < scene_corners_initial.size(); i++ )	// check the target region is inside the current image
	{
		if ( scene_corners_initial[i].x <= 0 &&
			 I_scene.cols < scene_corners_initial[i].x &&
			 scene_corners_initial[i].y <= 0 &&
			 I_scene.rows < scene_corners_initial[i].y )
			success = false;
	}

	/*
	 * 	-- step 6: visualize the results
	 */
	cv::namedWindow("SIFT Matches & Object detection");
	cv::Mat img_matches;
	cv::drawMatches( I_object, std::vector<cv::KeyPoint> (), I_scene, std::vector<cv::KeyPoint> (),
						std::vector<cv::DMatch> (), img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
						std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	for ( int i = 0; i < points_object.size(); i++ )
	{
		cv::Scalar color(255,0,0,0);
		if( inliers.size() != 0 )	// the same as !H.empty()
		{
			if ( inliers[i] == 1 )
				color = cv::Scalar(0,255,0,0);
			else
				color = cv::Scalar(255, 0, 0, 0);
		}
		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		//cv::line( img_matches, points_object[i], points_scene[i] + cv::Point2f( I_object.cols, 0), color, 1 );

	}
	//-- Draw the detected object in the scene image
	cv::line( img_matches, object_corners[0], object_corners[1], cv::Scalar(0,255,0), 1 );
	cv::line( img_matches, object_corners[1], object_corners[2], cv::Scalar(0,255,0), 1 );
	cv::line( img_matches, object_corners[2], object_corners[3], cv::Scalar(0,255,0), 1 );
	cv::line( img_matches, object_corners[3], object_corners[0], cv::Scalar(0,255,0), 1 );
	if( !H_pixel.empty() )
	{
		cv::line( img_matches, scene_corners_initial[0] + cv::Point2f( I_object.cols, 0), scene_corners_initial[1] + cv::Point2f( I_object.cols, 0), cv::Scalar(0,255,0), 1 );
		cv::line( img_matches, scene_corners_initial[1] + cv::Point2f( I_object.cols, 0), scene_corners_initial[2] + cv::Point2f( I_object.cols, 0), cv::Scalar(0,255,0), 1 );
		cv::line( img_matches, scene_corners_initial[2] + cv::Point2f( I_object.cols, 0), scene_corners_initial[3] + cv::Point2f( I_object.cols, 0), cv::Scalar(0,255,0), 1 );
		cv::line( img_matches, scene_corners_initial[3] + cv::Point2f( I_object.cols, 0), scene_corners_initial[0] + cv::Point2f( I_object.cols, 0), cv::Scalar(0,255,0), 1 );
	}

	cv::imshow("SIFT Matches & Object detection", img_matches);
	cv::imwrite("img_save/detection.png", img_matches);
	cv::waitKey(0);
	cv::destroyWindow( "SIFT Matches & Object detection" );

	/*
	 * 	-- step 7: release
	 */
	kdtree_release( kd_root );
	free( feat1 );
	free( feat2 );

	if ( success == false )
		return -1;
	else
		return 0;
}

/*
 * motion constraint verification using Homography decomposition
 */
int PlanarObjectDetection::planarMotionConstraintVerification(cv::Mat H)		// x = Hx', x and x' are image coordinates
																				// return 0 means content the constraint otherwise not content
{
	// decompose homography
	vpHomography aHb;	// initial as identity
	for( int i = 0; i < 3; i++ )
	{
		for(int j = 0; j < 3; j++)
		{
			aHb[i][j] = H.at<double>(i,j);
		}
	}

	vpRotationMatrix aRb  ;
	vpTranslationVector aTb ;
	vpColVector n ;
	aHb.computeDisplacement(aRb, aTb, n) ;

	vpThetaUVector thetau(aRb);
	std::cout << " thetau.x	" << thetau[0]*180/PI << std::endl;
	std::cout << " thetau.z	" << thetau[2]*180/PI << std::endl;

	if( std::abs(thetau[0]) < PI/18 && std::abs(thetau[2]) < PI/18 )	// no x and z axis rotations
	{
		return 0;
	}
	else
		return -1;
}

// converse points from pixel to meter
void PlanarObjectDetection::pixeltometer(const cv::Mat cameraMatrix,					// camera matrix
					  	  	  	  	  	  const	std::vector<cv::Point2f> points_pixel,	// points on image plane in pixel
					  	  	  	  	  	  std::vector<cv::Point3f> &points_meter)		// points on image plane in meter
{
	std::vector<cv::Point3f> points_pixel_homo;
	points_pixel_homo.reserve(4);
	for(int i = 0; i < points_pixel.size(); i++)
	{
		points_pixel_homo.push_back( cv::Point3f(points_pixel[i].x, points_pixel[i].y, 1.0f) );
	}
	// pixel meter conversion using camera model
	cv::Mat point_pixel_Mat = (cv::Mat_<double>(3,4) << points_pixel[0].x,
														points_pixel[1].x,
														points_pixel[2].x,
														points_pixel[3].x,
														points_pixel[0].y,
														points_pixel[1].y,
														points_pixel[2].y,
														points_pixel[3].y,
														1, 1, 1, 1);
	cv::Mat points_meter_Mat = cameraMatrix.inv()*point_pixel_Mat;
	points_meter.clear();			// important !!!
	for(int i = 0; i < points_pixel.size(); i++ )
	{
		cv::Vec3f vec = points_meter_Mat.col(i);
		cv::Point3f pt(vec[0], vec[1], vec[2]);
		// normalization
		pt.x = pt.x/pt.z;
		pt.y = pt.y/pt.z;
		pt.z = 1;
		points_meter.push_back(pt);
	}
}

// visual feature estimation: transform image points from frame x to frame y
// x refers to the remembered frame, y refers to the current frame
// by homography reconstruction: 3D structure of the target and the motion between the two frames
int PlanarObjectDetection::visualFeatureEstimation( const vpHomogeneousMatrix &y_T_x,				// the transformation matrix between two frames
													const vpPlane &x_Plane,							// 3D structure of the target plane
													const cv::Mat &cameraMatrix,					// camera intrinsic parameter
													PlanarObjectDetection &objectDetection)	// for the remembered target points
{
	// recover the homography matrix
	vpHomography y_H_x( y_T_x, x_Plane );
	vpCameraParameters cam( cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1),
	                        cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2));

	// forward project the remembered target points onto the normalized image plane in meter
	std::vector<cv::Point2f> x_ptsMeter;
	for( int i = 0; i < objectDetection.scene_corners_lost.size(); i++ )
	{
		double u, v;
		u = objectDetection.scene_corners_lost[i].x;
		v = objectDetection.scene_corners_lost[i].y;
		double x, y;
		vpPixelMeterConversion::convertPointWithoutDistortion( cam, u, v, x, y );
		x_ptsMeter.push_back(cv::Point2f(x,y));
	}

	// transform to the current normalized image plane
	cv::Mat H= (cv::Mat_<double>(3,3) << y_H_x[0][0], y_H_x[0][1], y_H_x[0][2],
										  y_H_x[1][0], y_H_x[1][1], y_H_x[1][2],
										  y_H_x[2][0], y_H_x[2][1], y_H_x[2][2]);

	std::vector<cv::Point2f> y_ptsMeter;
	cv::perspectiveTransform( x_ptsMeter, y_ptsMeter, H);

	// transform from meter to pixel
	objectDetection.estimated_corners_current.clear();
	objectDetection.estimated_corners_current_meter.clear();
	for( int i = 0; i < y_ptsMeter.size(); i++ )
	{
		double x, y;
		x = y_ptsMeter[i].x;
		y = y_ptsMeter[i].y;
		double u, v;
		vpMeterPixelConversion::convertPointWithoutDistortion( cam, x, y, u, v );
		objectDetection.estimated_corners_current.push_back(cv::Point(u,v));
		objectDetection.estimated_corners_current_meter.push_back( cv::Point3f(y_ptsMeter[i].x, y_ptsMeter[i].y, 1));
	}

	// calculate the gravity center
	momentGravityCenter(estimated_corners_current_meter, pt_gravityCenter_current_meter);
	momentGravityCenter(estimated_corners_current, pt_gravityCenter_current);

//	float x_g = 0;
//	float y_g = 0;
//	float u_g = 0;
//	float v_g = 0;
//	for ( int i = 0; i < estimated_corners_current_meter.size(); i++ )
//	{
//		x_g = x_g + estimated_corners_current_meter[i].x ;
//		y_g = y_g + estimated_corners_current_meter[i].y;
//		u_g = u_g + estimated_corners_current[i].x ;
//		v_g = v_g + estimated_corners_current[i].y;
//	}
//	x_g = x_g/4;
//	y_g = y_g/4;
//	u_g = u_g/4;
//	v_g = v_g/4;
//
//	pt_gravityCenter_current.x = u_g;
//	pt_gravityCenter_current.y = v_g;
//	pt_gravityCenter_current_meter.x = x_g;
//	pt_gravityCenter_current_meter.y = y_g;

	return 0;
}

// gravity center of an image object
int PlanarObjectDetection::momentGravityCenter( const std::vector<cv::Point2f> & pts,
												cv::Point2f &g)
{
//	// Define the contour of an object by a 5 clockwise vertices on a plane
//	vpPoint p;
//	std::vector<vpPoint> vec_p(pts.size()); // vector that contains the vertices of the contour polygon
//	for( int i = 0; i < pts.size(); i++ )
//	{
//		p.set_x(pts[i].x);	// coordinates in meters in the image plane
//		p.set_y(pts[i].y);
//		vec_p.push_back(p);
//	}
//	vpMomentObject obj(1); // Create an image moment object with 1 as
//	// maximum order (because only m00,m01,m10
//	// are needed to compute the gravity center primitive.
//	obj.setType(vpMomentObject::DENSE_POLYGON); // The object is defined by a countour polygon
//	obj.fromVector(vec_p); // Init the dense object with the polygon
//	vpMomentGravityCenter gc; // declaration of gravity center
//	gc.update(obj); // specify the object
//	gc.compute(); // compute the moment
//	g.x = gc.getXg();
//	g.y = gc.getYg();

	float x_g = 0;
	float y_g = 0;
	for ( int i = 0; i < pts.size(); i++ )
	{
		x_g = x_g + pts[i].x ;
		y_g = y_g + pts[i].y;
	}
	x_g = x_g/4;
	y_g = y_g/4;

	g.x = x_g;
	g.y = y_g;

	return 0;
}

int PlanarObjectDetection::momentGravityCenter( const std::vector<cv::Point3f> & pts,
												cv::Point3f &g)
{
//	// Define the contour of an object by a 5 clockwise vertices on a plane
//	vpPoint p;
//	std::vector<vpPoint> vec_p(pts.size()); // vector that contains the vertices of the contour polygon
//	for( int i = 0; i < pts.size(); i++ )
//	{
//		p.set_x(pts[i].x);	// coordinates in meters in the image plane
//		p.set_y(pts[i].y);
//		vec_p.push_back(p);
//	}
//	vpMomentObject obj(1); // Create an image moment object with 1 as
//	// maximum order (because only m00,m01,m10
//	// are needed to compute the gravity center primitive.
//	obj.setType(vpMomentObject::DENSE_POLYGON); // The object is defined by a countour polygon
//	obj.fromVector(vec_p); // Init the dense object with the polygon
//	vpMomentGravityCenter gc; // declaration of gravity center
//	gc.update(obj); // specify the object
//	gc.compute(); // compute the moment
//	g.x = gc.getXg();
//	g.y = gc.getYg();

	float x_g = 0;
	float y_g = 0;
	float z_g = 0;
	for ( int i = 0; i < pts.size(); i++ )
	{
		x_g = x_g + pts[i].x ;
		y_g = y_g + pts[i].y;
		z_g = z_g + pts[i].z;
	}
	x_g = x_g/4;
	y_g = y_g/4;
	z_g = z_g/4;

	g.x = x_g;
	g.y = y_g;
	g.z = z_g;

	return 0;
}

// object direction esitmation with respect to the robot frame
// using the object gravity center to estimate it direction
// image												robot
//
//	----------------> x									 ^ x
//	|									 				 |
//	|									 				 |
//	|									   				 |
// y 									y <---------------
int PlanarObjectDetection::objectDirectionEstimation( const cv::Point2f gc,	// object moment in image coordinate
													  float & theta)					// direction in robot frame
{
	// in the image coordinate
	theta = std::atan2(gc.x-0, 1);
//	// in the robot coordinate
//	theta = - theta;

	return 0;
}

//int PlanarObjectDetection::objectDirectionEstimation( const std::vector<cv::Point2f> pts, 	// visual features in image coordinate
//													  float & theta )						// direction in robot frame
//{
//	// in the image coordinate
//	theta = std::atan2(targetMoment.x-0);
//	// in the robot coordinate
//	theta = - theta;
//
//	return 0;
//}

