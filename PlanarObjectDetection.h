/*
 * PlanarObjectDetection.h
 *
 *  Created on: Nov 14, 2012
 *      Author: robot
 */

#ifndef PLANAROBJECTDETECTION_H_
#define PLANAROBJECTDETECTION_H_

#include <opencv2/opencv.hpp>
#include <visp/vpHomography.h>

class PlanarObjectDetection
{
public:
	PlanarObjectDetection();

	/*
	 * 	planar detection
	 */
	int objectDetection(const cv::Mat &I_object,			// image projection of object (reference image)
						const cv::Mat &I_scene, 			// image projection of scene	(initial image)
						const cv::Mat cameraMatrix);		// camera intrinsic parameters

	/*
	 * motion constraint verification
	 */
	int planarMotionConstraintVerification(cv::Mat H);		// x = Hx'

	/*
	 * 	detect the keypoints
	 */
	void keypointsDetect();

	/*
	 * 	calculate the descriptor
	 */
	void descriptorsCompute();

	/*
	 * 	matching descriptor vectors
	 */
	void descriptorMatch();

	/*
	 *  get the "good match" using the matching distance
	 */
	void getGoodMatch();

	/*
	 * 	localize the object
	 */
	void objectLocalisation();

	/**
	 *  pixel to meter
	 */
	static void pixeltometer(const cv::Mat cameraMatrix,						// camera matrix
					  	  	  const	std::vector<cv::Point2f> points_pixel,		// points on image plane in pixel
					  	  	  std::vector<cv::Point3f> &points_meter);			// points on image plane in meter

	/*
	 *  visual feature estimation
	 */
	// by homography reconstruction: 3D structure of the target and the motion between the two frames
	int visualFeatureEstimation(const vpHomogeneousMatrix &y_T_x,			// the transformation matrix between two frames
								const vpPlane &x_Plane,						// 3D structure of the target plane
								const cv::Mat &cameraMatrix,				// camera intrinsic parameter
								PlanarObjectDetection &objectDetection);	// for the remembered target points
	// gravity center of an image object
	int momentGravityCenter(const std::vector<cv::Point2f> & pts, cv::Point2f &g);
	int momentGravityCenter(const std::vector<cv::Point3f> & pts, cv::Point3f &g);

	// object direction esitmation
	int objectDirectionEstimation( const cv::Point2f gc, float & theta);
	int objectDirectionEstimation( const std::vector<cv::Point2f> pts, float & theta );


	/*
	 *  visualize the results
	 */

	/*
	 *  variable
	 */
	cv::Mat H;												// homography transformation from image projection of object
															// to image projection of scene
	cv::Mat H_pixel;										// homography relative to pixel coordinates

	std::vector<cv::Point2f> object_corners;				// the four corners of target in reference image in pixel
	std::vector<cv::Point3f> object_corners_meter;			// the four corners of target in reference image in meter
	std::vector<cv::Point2f> scene_corners_initial;			// the four corners of object in the initial scene image in pixel
	std::vector<cv::Point2f> scene_corners_current;			// the four corners of object in the current scene image in pixel
	std::vector<cv::Point2f> scene_corners_currentRight;	// the four corners of object in the current right scene image in pixel
	std::vector<cv::Point3f> scene_corners_current_meter;	// the four corners of object in the current scene image in meter

	std::vector<cv::Point2f> scene_corners_lost;			// the four corners of object in the lost scene image in pixel
															// used for refinding approach three

	std::vector<cv::Point2f> estimated_corners_current;		// the estimated four corners of the target in current image plane in pixel
	std::vector<cv::Point3f> estimated_corners_current_meter;	// the estimated four corners of the target in current normalized image plane in meter
	cv::Point2f	pt_gravityCenter_current;
	cv::Point3f	pt_gravityCenter_current_meter;
	// float theta;											// object direction in the camera frame

	std::vector<std::vector<cv::Point2f> > object_corners_trajectory;	// remember the trajectories of the target corners

	std::vector<cv::Point3f> scene_corners_3D;				// the four corners of object in 3D space in meter
	//std::vector<cv::Vec4f >	 scene_corners_4D;				// the homogeneous coordinates of the four corners of object in 3D space in meter
	cv::Mat 				 scene_corners_4D;
};


#endif /* PLANAROBJECTDETECTION_H_ */
