/*
 * Main.cpp
 *
 *  Created on: Oct 29, 2012
 *      Author: robot
 */

#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDot.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpHomography.h>
#include <visp/vpPose.h>
#include <visp/vpMeterPixelConversion.h>


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>

#include "RobotLink.h"

#include "LaserSensing.h"
#include "Navigation.h"
#include "demo_odo.h"
#include "visualOdometryMex.h"
#include "matcherMex.h"
#include "BSpline.h"
#include "motionControl.h"
#include "myDraw.h"
#include "myUtilities.h"
#include "PlanarObjectDetection.h"
#include "ESMlibry.h"


#define Save_Data
//#define Clusters_Pointer
//#define	Simulation
//#define Interal_Odometry
#define Visual_Odometry
#define Generate_Log

//using namespace cv;

static Matcher *M;			// later try to define as global variables
static VisualOdometry *VO;

int main(int argc, char** argv)
{
#ifndef Simulation
	RobotLink * rl = new RobotLink(NETWORK_ROBOT_IP);
	rl->setLeftVideoString(NETWORK_CAMERA_LEFT_IP, NETWORK_CAMERA_PORT, NETWORK_CAMERA_PATH);
	rl->setRightVideoString(NETWORK_CAMERA_RIGHT_IP, NETWORK_CAMERA_PORT, NETWORK_CAMERA_PATH);
	rl->toogleCamera( );
	rl->toogleLaser();
#endif

	cv::namedWindow("img", CV_WINDOW_AUTOSIZE);						// left image window
	cv::namedWindow("img2", CV_WINDOW_AUTOSIZE);					// right image window
	cv::namedWindow("I_tracking", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("laser", CV_WINDOW_AUTOSIZE);				// laser window
	cv::namedWindow( "M_Coordinate", CV_WINDOW_AUTOSIZE );		// odometry window
	cv::namedWindow("laser reactive obstacle",CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "Visual features estimation", CV_WINDOW_AUTOSIZE );	// visual servoing image plane
	cv::moveWindow("img", 20,0);
	cv::moveWindow("img2", 700,0);
	cv::moveWindow("I_tracking", 1350,20);
	cv::moveWindow("laser", 50,600);
	cv::moveWindow("laser reactive obstacle", 750,600);
	cv::moveWindow("M_Coordinate", 1350,600);
	//cv::moveWindow("Visual features estimation", 50,600);

#ifndef Simulation
	rl->setVlVa(0.0,0.0, false);
	//rl->setPanTilt(0,0,2000,2000,PANTILT_MODE_POSPANTILT);
	Poco::Thread::sleep(5000);
#endif

	int angle = 0; int sens = 1;

	// my variables
#ifdef Simulation
	std::string img_folder = "./img_save/";
#else
	std::string img_folder = "./img_save/";
#endif
	std::string img1_path;
	std::string img2_path;
	std::ostringstream s;
	int loopNumber = 0;

	// set the initial motion state
	MotionState motionState;
	motionState.motionStateCurrent 	  		= MotionState::TARGET_DETECTION;	// or set as OBSTACLE_AVOIDANCE
	motionState.motionStateRemembered		= MotionState::TARGET_DETECTION;
	motionState.b_rotateFinding		 		= false;
	motionState.b_rotateFindingStart  		= false;
	motionState.b_targetFindingDetection 	= false;	// if true, can do detection

	Robot robot( 	0.25,							// robot radius
					Pose(cv::Point2f(0,0),0,0),		// initial pose	in the global coordinate
					0,								// linear velocity
					0,								// angular velocity
					0.05,							// maximum linear velocity			0.4m/s
					20*PI/180,						// maximum angular velocity			20 deg/s
					0.2,							// maximum linear acceleration		0.4 m/s^2
					20*PI/180,						// maximum angular acceleration		20 deg/s^2
					2,								// minimum rotation radius			2 m
					0.5,							// maximum curvature of rotation	0.5/m
					0.5,							// distance between the boundary and the robot (the position of laser)	0.7
													// and following path with respect to the boundary of the obstacle
					1.0,							// the zone to reactive with obstacle, distance is between the boundaries	1.7
					0.35,							// the distance to stop
					0.1,							// maximum velocity during obstacle avoidance
					motionState);

	// demo window for laser data
//	int width_winLaser = 1000;
//	int height_winLaser = 1000;
	int width_winLaser = 500;	// window size
	int height_winLaser = 500;
	float width_area = 10;		// Cartesian coordinate
	float height_area = 10;

	unsigned time_0, time_current, time_elapsed;
	float	Vl_cruise = 0.1;				// as the maximum velocity
	float	Vl_0 = 0;							// the intial value for the linear velocity profile
	float	Vl_acceleration = 0.01;			// the stop acceleration is 3 times of this
//	bool b_start = false;					// the mode
//	bool b_stop = false;

	/*****************************************************************************\
	 * object detection															 *
	\*****************************************************************************/
	/*
	 *  load the target
	 */
	cv::Mat I1_object = cv::imread("I1_object.pgm", 0);
	if( I1_object.empty() )
	{
		std::cout << "an error with the image of object ... " << std::endl;
		return -1;
	}
	PlanarObjectDetection objectDetection;

	/*****************************************************************************\
	 * ESM tracking																 *
	\*****************************************************************************/
	// The tracking parameters
	// miter: the number of iterations of the ESM algorithm (>= 1);
	// mprec: the precision of the ESM algorithm (1..10)
	// low precision = 1, high precision = 10;
	// miter and mprec should be chosen depending on the available
	// computation time;
	// For low-speed PCs or high video framerate choose low miter and low mprec.
	int miter = 5,  mprec = 4;

	// The window position (upper left corner)
	// The image coordinates start from (0,0)
	int posx, posy;

	// The window size
	int sizx, sizy;

	// The image read / acquired
	imageStruct* I = NULL;

	// The global tracking structure
	trackStruct esmTrack;

	// tracking demo
	cv::Mat I_track;

	/**
	 *  pose estimation
	 */
	vpPoint P[4] ;	// To compute the pose, at least four points are required.
	vpPose pose ;
	// Set the 3D coordinates of the points in the target/object frame in meter
	// here the four points a coplanar, pay attention of the order
	P[0].setWorldCoordinates( 0, 0, 0 ) ;
	P[1].setWorldCoordinates( 0.558, 0, 0 ) ;
	P[2].setWorldCoordinates( 0.558, 0.735, 0 ) ;
	P[3].setWorldCoordinates( 0, 0.735, 0 ) ;

	/*****************************************************************************\
	 * visual servoing control													 *
	\*****************************************************************************/
	//vpCameraParameters cam(593.79260, 593.53509, 303.41645, 232.69472);
	//vpServo task;									// create a task
	//int nbpoint = 4;
	//vpImagePoint imgPt[nbpoint];					// image point in pixel
	//vpDot dot[nbpoint];								// tracked object
	//vpFeaturePoint feature_s[nbpoint], feature_sd[nbpoint];			// current and desired feature
	//double xd[nbpoint], yd[nbpoint], Zd[nbpoint];
	// float	vl_vs = 0.02;
	float	vl_vs = robot._vMax;
	float	va_vs = 0;

	/**
	 *  IJCU:
	 */
	VisualServoing visualServoing;
	visualServoing._Z_IJPU = 1;			// depth information
	visualServoing._lambd_IJPU = 0.5;	// gain

	/*****************************************************************************\
	 * target finding															 *
	\*****************************************************************************/
	double 	thetaFinding_ini = 0;				// the orientation at the beginning of the rotation
	double 	dthetaFinding = PI/6;				// the angular to rotate for next detection
	double 	thetaFinding_threshold = PI/20;		// threshold for stopping rotation
	double 	thetaFinding_va = PI/20;			// the angular velocity to find the target

	int 	motionState_remember = 0;
	int 	stop_number = 0;					// the number call stop state
	int 	stop_threshold = 10; 				// predefined times to call stop state

	/**
	 *  approach one
	 */
	// remember the informations when the target lost
	int numFrame_lostRemember = 0;			// remember the odometry frame number when the target lost
	vpHomogeneousMatrix w_T_c;				// transformation of current camera frame with respect to the world frame
	vpHomogeneousMatrix lost_T_target;		// remember the transformation of the target with respect to the camera frame
	vpHomogeneousMatrix w_T_lost;			// remember the transformation of the camera frame with respect to the world frame when lost

	/**
	 *  approach third: using homography
	 *  remember the target structure (planar parameters)
	 */
	std::vector<cv::Point2f> scene_corners_remembered;		// remembered target coordinates in the left camera image plane in pixel
	cv::Mat normalRemebered(3,1,CV_64F);					// remember the plane, method one: normal and perpendiculardistance
	double perpendicularDistance = 0;

	vpPlane lost_Plane;										// remember the plane, method two: three points

	/*****************************************************************************/
	/* odometry
	/*****************************************************************************/
	// visual odometry
	std::vector<Pose> Odometries;
	DemoOdo demo;
	demo.init("stereo","folder");
	// internal Odometry
	Pose internalOdometry_initial;

	/*****************************************************************************/
	/* stereo rectification
	/*****************************************************************************/
//	// after 2012_09_19
//	// intrinsic parameters
//	cv::Mat cameraMatrix_1 = (cv::Mat_<double>(3,3) << 593.19704, 		0, 292.73890,
//															  0, 593.07076, 224.55454,
//															  0,	     0,			1);
//	cv::Mat distCoeffs_1 = (cv::Mat_<double>(1,5) << -0.44931, 0.17271, 0.00159, 0.00411, 0.00000);
//	cv::Mat cameraMatrix_2 = (cv::Mat_<double>(3,3) << 596.47038, 		0, 304.79233,
//															  0, 597.92401, 213.82504,
//															  0,	     0,			1);
//	cv::Mat distCoeffs_2 = (cv::Mat_<double>(1,5) << -0.50426, 0.33753, 0.00127, 0.00229, 0.00000);
//
//	// extrinsic parameters
//	cv::Mat r = (cv::Mat_<double>(1,3) << -0.00688, 0.02052, -0.02026);
//	cv::Mat R;
//	cv::Rodrigues(r,R);
//	cv::Mat T = (cv::Mat_<double>(3,1) << -0.08002273, 0.00042560, 0.00214352);		// in metric

//	// after 2012_10_04
//	// intrinsic parameters
//	cv::Mat cameraMatrix_1 = (cv::Mat_<double>(3,3) << 593.79260, 		0, 303.41645,
//															  0, 593.53509, 232.69472,
//															  0,	     0,			1);
//	cv::Mat distCoeffs_1 = (cv::Mat_<double>(1,5) << -0.46224, 0.21688, -0.00131, 0.00055, 0.00000);
//	cv::Mat cameraMatrix_2 = (cv::Mat_<double>(3,3) << 598.46916, 		0, 328.60670,
//															  0, 597.78216, 226.26846,
//															  0,	     0,			1);
//	cv::Mat distCoeffs_2 = (cv::Mat_<double>(1,5) << -0.46486, 0.22200, -0.00148, -0.00121, 0.00000);
//
//	// extrinsic parameters
//	cv::Mat r = (cv::Mat_<double>(1,3) << 0.00213, -0.00111, -0.01900);
//	cv::Mat R;
//	cv::Rodrigues(r,R);
//	cv::Mat T = (cv::Mat_<double>(3,1) << -0.07977557, 0.00085184, 0.00033989);		// in metric

	// after 2013_03_30
	// intrinsic parameters
	cv::Mat cameraMatrix_1 = (cv::Mat_<double>(3,3) << 594.35962, 		0, 301.27439,
															  0, 595.62007, 229.30863,
															  0,	     0,			1);
	cv::Mat distCoeffs_1 = (cv::Mat_<double>(1,5) << -0.46688, 0.22455, 0.00056, 0.00117, 0.00000);
	cv::Mat cameraMatrix_2 = (cv::Mat_<double>(3,3) << 596.41898, 		0, 322.57216,
															  0, 597.65558, 223.41411,
															  0,	     0,			1);
	cv::Mat distCoeffs_2 = (cv::Mat_<double>(1,5) << -0.46449, 0.21790, -0.00060, -0.00026, 0.00000);

	// extrinsic parameters
	cv::Mat r = (cv::Mat_<double>(1,3) << 0.00272, 0.00199, -0.01946);
	cv::Mat R;
	cv::Rodrigues(r, R);
	cv::Mat T = (cv::Mat_<double>(3,1) << -0.07930486, 0.00103221, -0.00079854);		// in metric

	// stereo rectification
	// rectify images
	cv::Mat R1, R2, P1,P2, Q;
	cv::Rect validPixROI1, validPixROI2;
	cv::Size imageSize(640,480);
	cv::Mat rmap[2][2];
	demo.stereoRectification(cameraMatrix_1, distCoeffs_1, cameraMatrix_2, distCoeffs_2, imageSize, R, T, R1, R2, P1, P2, Q, imageSize, validPixROI1,  validPixROI2, rmap);
	cv::Mat cameraMatrix_1_rectified = (cv::Mat_<double>(3,3) << P1.at<double>(0,0), 				  0, P1.at<double>(0,2),
																  	  	  	  	  0, P1.at<double>(1,1), P1.at<double>(1,2),
																  	  	  	  	  0,	     		  0,				 1);
	cv::Mat cameraMatrix_2_rectified = (cv::Mat_<double>(3,3) << P2.at<double>(0,0), 				  0, P2.at<double>(0,2),
																	  	  	  	  0, P2.at<double>(1,1), P2.at<double>(1,2),
																	  	  	  	  0,	     		  0,				 1);

	// odometry objects
	double f     = P1.at<double>(0,0);
	double cu    = P1.at<double>(0,2);
	double cv    = P1.at<double>(1,2);
	double base  = -P2.at<double>(0,3)/P2.at<double>(0,0);			// here is for stereo
	double pitch = 0;

	demo.initCalib(f, cu, cv, base);

	// matching parameters
	int nms_n = 3;					// non-max-suppression: min. distance between maxima (in pixels)
	int nms_tau = 50;				// non-max-suppression: interest point peakiness threshold
	int match_binsize = 40;			// matching bin width/height (affects efficiency only)
	int match_radius = 200;			// matching radius (du/dv in pixels)
	int match_disp_tolerance = 1;   // du tolerance for stereo matches (in pixels)
//	int outlier_disp_tolerance = 5; // outlier removal: disparity tolerance (in pixels)
//	int outlier_flow_tolerance = 5; // outlier removal: flow tolerance (in pixels)
	int outlier_disp_tolerance = 5; // outlier removal: disparity tolerance (in pixels)
	int outlier_flow_tolerance = 5; // outlier removal: flow tolerance (in pixels)
	int multi_stage = 1;			// 0=disabled,1=multistage matching (denser and faster)
	int half_resolution = 1;		// 0=disabled,1=match at half resolution, refine at full resolution
	int refinement = 1;				// refinement (0=none,1=pixel,2=subpixel)

	// bucketing parameters
	int max_features = 10;
	float bucket_width = 20;
	float bucket_height = 20;
	float deltaT = 0.1f;

	// init matcher
	MatcherMex matcherMex;
	matcherMex.matcherMex( "init", nms_n,nms_tau,match_binsize,match_radius,match_disp_tolerance,
							outlier_disp_tolerance,outlier_flow_tolerance,multi_stage,half_resolution,refinement );

	// init odometry objects
	VO = new VisualOdometry();
	VO->setCalibration(f,cu,cv,base,pitch);
	VisualOdometryMex visualOdometryMex;

	// init transformation matrix array
	std::vector<Matrix> Tr_total;
	Matrix matrix(4,4);
	matrix.eye();
	Tr_total.push_back (matrix);

	// demo visual odometry
	cv::Mat M1_origi, M2_origi;
	cv::Mat M1, M2;
	cv::Mat M1_color, M2_color;
	int width_winOdo = 500;
	int height_winOdo = 500;
	cv::Mat M_Coordinate(cv::Size(width_winOdo,height_winOdo),CV_8UC3);
	float range_x = 14;
	float range_z = 14;
	double ratioW_draw = M_Coordinate.cols/range_x;	// width for x meters
	double ratioH_draw = M_Coordinate.rows/range_z;	// heighth for y meters
	cv::Point point_center( M_Coordinate.cols/2, M_Coordinate.rows/2 );	 // image coordinate
	demo.plotCoordinate( M_Coordinate, point_center, ratioW_draw, ratioH_draw, range_x, range_z );

	// draw laser
	cv::Mat I_laser(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
	cv::Mat I_laserNoiseFiltering(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//	cv::Mat I_laserClustering(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//	cv::Mat I_laserMerging(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
	cv::Mat I_laserSegments(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
	cv::Mat I_laserClusteringAndMerging(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
	cv::Mat I_laserPolygonalChains(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
	cv::Mat I_laserConvexChains(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
	cv::Mat I_laserReactiveObstacle(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
	/*
	 *  myDraw
	 */
	MyDraw myDraw;
	int radius_naviDraw = 5000;	// visualize a range of 5 meter
	myDraw._wScale_meter2laserwindow = width_winLaser/(2*5);
	myDraw._hScale_meter2laserwindow = height_winLaser/(2*5);

	// navigation
	Navigation navi;
	navi.initLaser(1000, 0.006132813, 5000, M_Coordinate);

	// laser sensing
	LaserSensing laserSensing;


	// for transformation
	int failure = 0;		// failure = n : count the times failed, failure == 0 not failed
	bool success = NULL;
	int k = 1;	// the index of Tr_total (transformation matrix)
	std::stringstream ss;

	// motion control
	motionControl control;
	control._preavoidDistance = 1.0;	// 1.0 m
	control._s_path = 0;
	control._predictionMethod.method = PredictionMethod::STRAIGHT_LINE_FOLLOWING;
	control._b_clockwise = false;		// this flag should be decide based on laser data
	control._b_leave = false;
	control._b_collision = false;

	/*****************************************************************************/
	/* velocity
	/*****************************************************************************/
	float expectedVa = 0;
	float expectedVl = 0;
	float _newVa = 0;
	float _newVl = 0.0;

	/*****************************************************************************/
	/* visual feature estimation
	/*****************************************************************************/
	cv::Mat I_featureEstimation;
	cv::Point pt_featureEstimated[4];

	/*****************************************************************************\
	 * Log																		 *
	\*****************************************************************************/
#ifdef Generate_Log
	/*
	 *  defination
	 */
	std::string fVisualOdometry = "result/visualOdometry.txt";
	std::string fRobotPose 		= "result/robotPose.txt";			// the laser pose (x,y,theta)
	std::string fTargetInCamera	= "result/targetInCamera.txt";		// the target coordinate in 3D, with respect to camera when start obstacle avoidance
																	// x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3
	std::string fControlInput	= "result/controlInput.txt";		// v, w (degree/s)
	std::string fRobotState		= "result/robotState.txt";			// integer: target finding, obstacle avoidance, ...
	std::string fCameraInWorld	= "result/cameraInWorld.txt";		// Homogeneous Matrix	w_T_c
	std::string fLaserData		= "result/laserData.txt";			// laser data with noise filtered
	std::string fLaserFiltered	= "result/laserDataFiltered.txt";	// laser data with noise filtered
	std::string fLaserObstacle	= "result/laserObstacle.txt";		// laser data with selected obstacle
	std::string ftargetFeatures	= "result/targetFeatures.txt";		// current targets features
	std::string fBsplinesDatapts= "result/fBsplinesDatapts.txt";	// current B-spline data points
	std::string fBspline		= "result/Bspline.txt";				// current B-spline

	/*
	 *  write the headerline
	 */
	std::ofstream lRobotPose;
	lRobotPose.open(fRobotPose.c_str());
	lRobotPose << "x" << "\t" << "y" << "\t" << "theta" << std::endl;

	std::ofstream lTargetInCamera;
	lTargetInCamera.open(fTargetInCamera.c_str());
	lTargetInCamera << "x0" << "\t" << "y0" << "\t" << "z0" << "\t" << "x1" << "\t" << "y1" << "\t" << "z1" << "\t"
					<< "x2" << "\t" << "y2" << "\t" << "z2" << "\t" << "x3" << "\t" << "y3" << "\t" << "z3" << std::endl;

	std::ofstream lControlInput;
	lControlInput.open(fControlInput.c_str());
	lControlInput << "v" << "\t" << "w" << std::endl;

	std::ofstream lRobotState;
	lRobotState.open(fRobotState.c_str());

	std::ofstream lCameraInWorld;
	lCameraInWorld.open(fCameraInWorld.c_str());
	lCameraInWorld << "T00" << "\t" << "T01" << "\t" << "T02" << "\t" << "T03" << "\t"
					<< "T10" << "\t" << "T11" << "\t" << "T12" << "\t" << "T13" << "\t"
					<< "T20" << "\t" << "T21" << "\t" << "T22" << "\t" << "T23" << "\t"
					<< "T30" << "\t" << "T31" << "\t" << "T32" << "\t" << "T33" <<  std::endl;

//	std::ofstream lTargetInCamera;
//	lTargetInCamera.open(fTargetInCamera.c_str());
//	lTargetInCamera << "T00" << "\t" << "T01" << "\t" << "T02" << "\t" << "T03" << "\t"
//					<< "T10" << "\t" << "T11" << "\t" << "T12" << "\t" << "T13" << "\t"
//					<< "T20" << "\t" << "T21" << "\t" << "T22" << "\t" << "T23" << "\t"
//					<< "T30" << "\t" << "T31" << "\t" << "T32" << "\t" << "T33" <<  std::endl;

	std::ofstream lLaserData;
	lLaserData.open(fLaserData.c_str());
	lLaserData << "angularResolution" << "\t" << "stepNumber" << "\t" << "rangeScale" << "\t"  << "Data" << std::endl;

	std::ofstream lLaserFiltered;
	lLaserFiltered.open(fLaserFiltered.c_str());
	lLaserFiltered << "LaserDataFiltered" << std::endl;

	std::ofstream lLaserObstacle;
	lLaserObstacle.open(fLaserObstacle.c_str());
	lLaserObstacle << "LaserObstacle" << std::endl;

	std::ofstream lTargetFeatures;
	lTargetFeatures.open(ftargetFeatures.c_str());
	lTargetFeatures << "x0" << "\t" << "y0" << "\t" << "x1" << "\t" << "y1" << "\t"
					<< "x2" << "\t" << "y2" << "\t" << "x3" << "\t" << "y3" << "\t" << std::endl;

	std::ofstream lBsplinesDatapts;
	lBsplinesDatapts.open(fBsplinesDatapts.c_str());
	lBsplinesDatapts << "data points (xi, yi)" << std::endl;

	std::ofstream lBspline;
	lBspline.open(fBspline.c_str());
	lBspline << "data points (xi, yi)" << std::endl;


#endif		//#ifdef Generate_Log

	while(true)
	{
		// send the velocity
		//rl->setPanTilt(0,0,2000,2000,PANTILT_MODE_POSPANTILT);

		//std::cout << "Laser : " << rl->IsLaserGoing() << std::endl;
		try
		{
			// start the time clock
			clock_t t1,t2;
			t1 = clock();
			////////////////////////////////////////////////////////////////////////
			//	start the whole navigation part
			////////////////////////////////////////////////////////////////////////
#ifndef Simulation
			LaserScan * las = rl->getLaserScan();
			Image * img = rl->getLeftCameraFrame();
			Image * img2 = rl->getRightCameraFrame();
			Odometry * internalOdometry = rl->getOdometry();
			if( las!=NULL && img!=NULL && img2!=NULL && internalOdometry!=NULL )	// if all the data are not NULL
			{
				if ( las->data!=NULL && las->size>0 && img->data!=NULL && img2->data!=NULL)		// if all the data are ok
				{
					if( loopNumber == 0 )	// at the beginning
					{
						//  set the parameter of LaserSensing object

						laserSensing.angularResolutionSet(0.006132813);
						laserSensing.maximumRadiusSet(4095);
						laserSensing.rangeScaleSet(1000);
						laserSensing.scanAnglSet(laserSensing.angularResolutionGet()*(double)las->size);
						laserSensing.stepNumberSet(las->size);

						myDraw.angularResolutionSet(0.006132813);
						myDraw.maximumRadiusSet(4095);
						myDraw.rangeScaleSet(1000);
						myDraw.scanAnglSet(laserSensing.angularResolutionGet()*(double)las->size);
						myDraw.stepNumberSet(las->size);
						myDraw.radiusNaviSet(radius_naviDraw);

					}	// at the beginning if( loopNumber == 0 )

					/**
					 * 	1. for each loop, transform the data form
					 */

					/**
					 *  1.1 transform the left image
					 */
					// transform the left image to the default format of OpenCV
					CvSize size = cvSize(img->width, img->height);
					int depth  = IPL_DEPTH_8U;
					int channels = 3;
					IplImage *frame = cvCreateImageHeader(size, depth, channels);	// rgb
					IplImage *bgr   = cvCreateImage(size, depth, channels);			// default format of OpenCV
					frame->imageData = img->data;
					cvCvtColor(frame,bgr,CV_RGB2BGR);

					// convert IplImage to Mat
					cv::Mat M1_tempColor(bgr);
					cv::Mat M1_temp;
					cv::cvtColor(M1_tempColor, M1_temp, CV_BGR2GRAY);
					M1_temp.copyTo(M1_origi);												// get the original gray image

					// transform the image
					cv::remap(M1_origi, M1, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
					cv::cvtColor(M1, M1_color, CV_GRAY2BGR);

//#ifdef Save_Data	// save images from the left camera
//					s.str("");
//					s << "I1_" << std::setw(6) << std::setfill('0') << loopNumber << ".pgm";
//					img1_path = img_folder + s.str();
//					cv::imwrite( img1_path, M1);
//#endif				// save image

					// release the unnecessary image
					cvReleaseImage(&bgr);
					cvReleaseImageHeader(&frame);

					/**
					 *  1.2 transform the right image
					 */
					// transform the left image to the default format of OpenCV
					frame = cvCreateImageHeader(size, depth, channels);
					bgr   = cvCreateImage(size, depth, channels);
					frame->imageData = img2->data;
					cvCvtColor(frame,bgr,CV_RGB2BGR);

					// convert IplImage to Mat
					cv::Mat M2_tempColor(bgr);
					cv::Mat M2_temp;
					cv::cvtColor(M2_tempColor, M2_temp, CV_BGR2GRAY);

					// transform the image
					M2_temp.copyTo(M2_origi);												// get the original gray image
					cv::remap(M2_origi, M2, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
					cv::cvtColor(M2, M2_color, CV_GRAY2BGR);

//#ifdef Save_Data	// save images from the right camera
//					s.str("");
//					s << "I2_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
//					img2_path = img_folder + s.str();
//					//img2_path = s.str();
//					cv::imwrite( img2_path, M2);
//#endif				// save image

					// release the unnecessary image
					cvReleaseImage(&bgr);
					cvReleaseImageHeader(&frame);

					/**
					 *	2. do laser scan processing for each loop
					 */
					/*
					 *	Level One - part one: obstacle detection using laser range finder
					 */
//					std::vector<int> status;
//					std::vector<std::vector<int> >  Clusters;
//					std::vector<std::set<int> >  PolygonalChains;
//					std::vector<std::set<int> >  ConvexChains;
					//laserSensing.obstacleDetection(las, status, Clusters,PolygonalChains);
					laserSensing.obstacleDetection(las, laserSensing.status, laserSensing._Segments, laserSensing.Clusters, laserSensing.PolygonalChains, laserSensing.ConvexChains);

//					// draw laser results
//					I_laser = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//					myDraw.drawLaserCoordinate(width_area, height_area, I_laser);
//					myDraw.drawLaser(las, width_area, height_area, I_laser);							// raw data
//					I_laserNoiseFiltering = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//					myDraw.drawLaserCoordinate(width_area, height_area, I_laserNoiseFiltering);
//					myDraw.drawLaser(las, width_area, height_area, laserSensing.status, I_laserNoiseFiltering);	// noise filtered
//					I_laserClustering = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//					myDraw.drawLaserCoordinate(width_area, height_area, I_laserClustering);
//					myDraw.drawLaser(las, width_area, height_area, laserSensing.Clusters, I_laserClustering);		// clustering
//					I_laserMerging = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//					myDraw.drawLaserCoordinate(width_area, height_area, I_laserMerging);
//					myDraw.drawLaser(las, width_area, height_area, laserSensing.Clusters, I_laserClustering);		// clustering
//
//					I_laserPolygonalChains = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//					I_laserClustering.copyTo(I_laserPolygonalChains);
//					//myDraw.drawLaserCoordinate(width_area, height_area, I_laserPolygonalChains);
//					//myDraw.drawLaser(las, width_area, height_area, laserSensing.Clusters, laserSensing.ConvexChains, I_laserPolygonalChains);	// Polygonal Chains
//					myDraw.drawPolygonalChains( las, width_area, height_area, laserSensing.Clusters, laserSensing.PolygonalChains, I_laserPolygonalChains);
//					myDraw.drawConvexChains( las, width_area, height_area, laserSensing.Clusters, laserSensing.ConvexChains, I_laserPolygonalChains);

//					// using opencv function
//					laserSensing.obstacleDetection( las, laserSensing.status, laserSensing.Clusters, laserSensing.contourCurves,
//													laserSensing.polygonalCurves, laserSensing.convexCurves);
//					myDraw.drawContourCurves( las, width_area, height_area, laserSensing.contourCurves, I_laserPolygonalChains);
//					myDraw.drawPolygonalCurves( las, width_area, height_area, laserSensing.polygonalCurves, I_laserPolygonalChains);
//					myDraw.drawConvexCurves( las, width_area, height_area, laserSensing.convexCurves, I_laserPolygonalChains);

//#ifdef Generate_Log
//					for (int i=0; i < laserSensing.status.size(); i++ )
//						lLaserFiltered << laserSensing.status[i] << "\t";
//					lLaserFiltered << std::endl;
//#endif

					/*
					 *	visual odometry for each loop
					 */
					std::cout << " processing stereo images for odometry " << loopNumber << std::endl;
					Pose odometry;
#ifdef Simulation
					// read current stereo images
					s.str("");
					s << std::setfill('0') << std::setw(6) << loopNumber;
					img1_path = img_folder + "I1_" + s.str() + ".png";
					std::cout << ss << std::endl;
					FILE *pFile;
					if( pFile = fopen( img1_path.c_str(), "r" ) )
					{
						fclose(pFile);
						M1_origi = cv::imread( img1_path, 0 );
						cv::remap(M1_origi, M1, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
						cv::cvtColor( M1, M1_color, CV_GRAY2BGR);
					}
					else
					{
						std::cout << "Error reading current left image ..." << std::endl;
						break;
					}
					s.str("");
					s << std::setfill('0') << std::setw(6) << loopNumber;
					img2_path = img_folder + "I2_" + s.str() + ".png";
					if(  pFile = fopen( img2_path.c_str(), "r" ) )
					{
						fclose(pFile);
						M2_origi = cv::imread( img2_path, 0 );
						cv::remap(M2_origi, M2, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
						cv::cvtColor( M2, M2_color, CV_GRAY2BGR);
					}
					else
					{
						std::cout << "Error reading current right image ..." << std::endl;
						break;
					}
#endif
#ifdef Visual_Odometry
					// transform opencv image structure to libviso2 image structure
					// get pointers to input data
					uint8_t* I1          = (uint8_t*)M1.ptr(0);	// returns pointer to 0-th row
					int32_t I1_size[2] = { M1.cols, M1.rows };
					const int32_t *dims1 = I1_size;
					uint8_t* I2          = (uint8_t*)M2.ptr(0);	// returns pointer to 0-th row
					int32_t I2_size[2] = { M2.cols, M2.rows };
					const int32_t *dims2 = I2_size;

					// keep previous stereo images, if visual odometry estimate failed
					if ( failure == 0 )		// if not failed
						matcherMex.matcherMex( "push", I1, dims1, I2, dims2 );
					else
						matcherMex.matcherMex( "replace", I1, dims1, I2, dims2 );

					// start matching after reading 2nd stereo frames
					if ( loopNumber > 1 )
					{
						// match features and also get a bucketed version
						// matching method: 0 = flow, 1 = stereo, 2 = quad matching
						matcherMex.matcherMex( "match", 2);
						std::vector<Matcher::p_match> matches_full;	// full matches
						matcherMex.matcherMex( matches_full, "getmatches", 2 );
						std::vector<Matcher::p_match> matches_bucketing;	// bucketing matches
						matcherMex.matcherMex( "bucketing", max_features,bucket_width,bucket_height);
						matcherMex.matcherMex( matches_bucketing, "getmatches", 2 );

						// estimate egomotion, failure+1 is the number of frames between matches
						success = visualOdometryMex.visualOdometryMex( VO, "update", (failure+1)*deltaT, matches_bucketing, "stereo" );

						// get inliers
						std::vector<int32_t> inliers = VO->getInliers();

						// if visual odometry estimate was successful
						Matrix Tr(4,4);
						Tr.eye();	// define an identity matrix
						if ( success == true )
						{
							 // get transformation matrix (state)
							 // get state
							 //Tr = ~VO->getTransformation();	 // VO->getTransformation() return the transformation matrix of the previous frame
								//							 // respect to the current frame
								//							 // ~ to get the transform matrix of Tr also the previous frame respect to the current frame
							Tr = VO->getTransformation();		// it is better not use transform ~, it already output normal format
							 failure = 0;
						}
						// otherwise keep current position
						// (this happens if motion is very small)
						else
						{
							failure = failure+1;
						}

						Tr.inv();								// transformation matrix of the current frame respect to the previous frame
						// accumulate total transformation
						Tr_total.push_back( Tr_total[k-1]*Tr );	// Tr_total[k-1]*Tr get the transformation matrix of the current frame respect to the first frame


		//				std::cout << "Tr_total(current) :"  << std::endl;
		//				for(int i = 0; i < 4; i++)
		//				{
		//					for(int j = 0; j < 4; j++)
		//					{
		//						std::cout << Tr_total[Tr_total.size()-1].val[i][j] << "	  ";
		//					}
		//					std::cout << std::endl;
		//				}

						k=k+1;

						// draw the results
						demo.plotMatch( M1_color, matches_bucketing, 0);
						//demo.plotMatch( M1_color, matches_bucketing, inliers, 0);
						//demo.plotTrajectory( M_Coordinate, Tr_total[Tr_total.size()-1],point_center, ratioW_draw, ratioH_draw, range_x, range_z );

						// calculate odometry
						cv::Mat rotationMatrix = (cv::Mat_<double>(3,3) <<
						Tr_total[Tr_total.size()-1].val[0][0], Tr_total[Tr_total.size()-1].val[0][1], Tr_total[Tr_total.size()-1].val[0][2],
						Tr_total[Tr_total.size()-1].val[1][0], Tr_total[Tr_total.size()-1].val[1][1], Tr_total[Tr_total.size()-1].val[1][2],
						Tr_total[Tr_total.size()-1].val[2][0], Tr_total[Tr_total.size()-1].val[2][1], Tr_total[Tr_total.size()-1].val[2][2]);

						cv::Mat rotationVector;
						cv::Rodrigues(rotationMatrix,rotationVector);

		//				std::cout << " odo_z "	<< Tr_total[Tr_total.size()-1].val[2][3] << "odo_x "	<< Tr_total[Tr_total.size()-1].val[0][3]
		//				                  	     << " odo_phi "<< (float)rotationVector.at<double>(0,1)*180/PI << std::endl;

						odometry.p.x = Tr_total[Tr_total.size()-1].val[2][3];	// z
						odometry.p.y = -Tr_total[Tr_total.size()-1].val[0][3];	// -x
						odometry.phi = -(float)rotationVector.at<double>(0,1);	// ry, in robot coordinate rotate to the right if more than zero
						Odometries.push_back(odometry);

						// set the homogeneous matrix of the current camera frame
//						vpMatrix rotationM(3,3);
//						rotationM[0][0] = Tr_total[Tr_total.size()-1].val[0][0]; rotationM[0][1] = Tr_total[Tr_total.size()-1].val[0][1]; rotationM[0][2] = Tr_total[Tr_total.size()-1].val[0][2];
//						rotationM[1][0] = Tr_total[Tr_total.size()-1].val[1][0]; rotationM[1][1] = Tr_total[Tr_total.size()-1].val[1][1]; rotationM[1][2] = Tr_total[Tr_total.size()-1].val[1][2];
//						rotationM[2][0] = Tr_total[Tr_total.size()-1].val[2][0]; rotationM[2][1] = Tr_total[Tr_total.size()-1].val[2][1]; rotationM[2][2] = Tr_total[Tr_total.size()-1].val[2][2];
						vpRotationMatrix R;
						R[0][0] = Tr_total[Tr_total.size()-1].val[0][0]; R[0][1] = Tr_total[Tr_total.size()-1].val[0][1]; R[0][2] = Tr_total[Tr_total.size()-1].val[0][2];
						R[1][0] = Tr_total[Tr_total.size()-1].val[1][0]; R[1][1] = Tr_total[Tr_total.size()-1].val[1][1]; R[1][2] = Tr_total[Tr_total.size()-1].val[1][2];
						R[2][0] = Tr_total[Tr_total.size()-1].val[2][0]; R[2][1] = Tr_total[Tr_total.size()-1].val[2][1]; R[2][2] = Tr_total[Tr_total.size()-1].val[2][2];
						vpTranslationVector t(Tr_total[Tr_total.size()-1].val[0][3],Tr_total[Tr_total.size()-1].val[1][3],Tr_total[Tr_total.size()-1].val[2][3]);
						w_T_c.buildFrom(t,R);

						//std::cout << " x "	<< odometry.p.x << "y "	<< odometry.p.y << "phi "<< odometry.phi*180/PI << std::endl;
					}
					else
					{
						odometry.p.x = 0;
						odometry.p.y = 0;
						odometry.phi = 0;
						Odometries.push_back(odometry);
					}
#endif
					//demo.plotTrajectory( M_Coordinate, odometry,point_center, ratioW_draw, ratioH_draw, range_x, range_z );
					M_Coordinate = cv::Mat( cv::Size(width_winOdo,height_winOdo), CV_8UC3, cv::Scalar::all(255) );
					demo.plotCoordinate( M_Coordinate, point_center, ratioW_draw, ratioH_draw, range_x, range_z );
					demo.plotTrajectory( M_Coordinate, Odometries, point_center, ratioW_draw, ratioH_draw, range_x, range_z );


					/**
					 * 	3. Object detection using SIFT while robot keep static
					 */
					if ( robot._motionState.motionStateCurrent == MotionState::TARGET_DETECTION )
					{
						if( objectDetection.objectDetection(I1_object, M1, cameraMatrix_1_rectified) != 0 )	// object detection failed
						{
							std::cout << "can not detect the object in the current scene ... " << std::endl;
							robot._motionState.motionStateCurrent 	 	= MotionState::TARGET_FINDING;		// do not need stop, because TARGET_DETECTION already stop the robot
							robot._motionState.motionStateRemembered 	= MotionState::TARGET_DETECTION;
							robot._motionState.b_rotateFinding		 	= true;
							robot._motionState.b_rotateFindingStart	 	= true;

							// return -1;		// later will deal with unexcepted
						}
						else		// object detection successful
						{

							std::cout << "Object found ... " << std::endl;
							std::cout << "H = " << std::endl <<      objectDetection.H  << std::endl << std::endl;
							std::cout << "scene corners = " << std::endl;
							std::cout << "objectDetection.object_corners.size()" <<	objectDetection.object_corners.size() << std::endl;

							robot._motionState.motionStateCurrent = MotionState::TARGET_TRACKING;		// if detected, track the target

							/*
							 * 	initial the ESM tracker
							 */
							// The window position (upper left corner)
							// The image coordinates start from (0,0)
							posx = objectDetection.scene_corners_initial[0].x;
							posy = objectDetection.scene_corners_initial[0].y;
//							posx = std::max(objectDetection.scene_corners[0].x, objectDetection.scene_corners[3].x) + 10;
//							posy = std::max(objectDetection.scene_corners[0].y,objectDetection.scene_corners[1].y) + 10;
							// The window size
							sizx = std::min( objectDetection.scene_corners_initial[1].x, objectDetection.scene_corners_initial[2].x ) - posx;
							sizy = std::min( objectDetection.scene_corners_initial[2].y, objectDetection.scene_corners_initial[3].y ) - posy;
//							sizx = std::min( objectDetection.scene_corners[1].x, objectDetection.scene_corners[2].x ) - posx -10;
//							sizy = std::min( objectDetection.scene_corners[2].y, objectDetection.scene_corners[3].y ) - posy -10;

							// convert the image format
							// Mat -> imageStruct
//							IplImage I_Ipl;
//							I_Ipl = M1.operator _IplImage();
//							IplImage2imageStruct ( &I_Ipl, &I);
							Mat2imageStruct ( M1, &I);

							//Memory allocation for the tracking
							if ( MallTrack (&esmTrack, I, posx, posy, sizx, sizy, miter, mprec) )
								return (1);
							else
								std::cout << "ESM Tracking structure ready\n";

							/**
							 *  show the initialisation result
							 */
							I_track = M1_color.clone();
							template_mark( I_track, objectDetection.scene_corners_initial );
							cv::rectangle(I_track, cv::Rect(posx,posy, sizx,sizy), cv::Scalar(0,0,255), 1, 8, 0);	// draw tracking region
							// add the warped target in the first frame image
							imageStruct* pattern_c = GetPatr (&esmTrack);
							int x_pattern_dsp = 5;
							int y_pattern_dsp = 5;
//							I_Ipl = I_track.operator _IplImage();
//							for(int i = 0; i< pattern_c->rows; i++)
//							{
//								for(int j = 0; j< pattern_c->cols; j++)
//								{
//									int ii = i + y_pattern_dsp;
//									int jj = j + x_pattern_dsp;
//									(I_Ipl.imageData + I_Ipl.widthStep*ii)[jj*I_Ipl.nChannels]   = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
//									(I_Ipl.imageData + I_Ipl.widthStep*ii)[jj*I_Ipl.nChannels+1] = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
//									(I_Ipl.imageData + I_Ipl.widthStep*ii)[jj*I_Ipl.nChannels+2] = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
//								}
//							}
							for(int i = 0; i< pattern_c->rows; i++)
							{
								for(int j = 0; j< pattern_c->cols; j++)
								{
									int ii = i + y_pattern_dsp;
									int jj = j + x_pattern_dsp;
									I_track.at<cv::Vec3b>(ii,jj)[0]  = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
									I_track.at<cv::Vec3b>(ii,jj)[1]  = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
									I_track.at<cv::Vec3b>(ii,jj)[2]  = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
								}
							}

							// draw a white square around tracking pattern
							cv::rectangle(M1, cv::Rect(x_pattern_dsp-1,y_pattern_dsp-1, pattern_c->cols,pattern_c->rows), cv::Scalar(255,255,255), 2, 8, 0);
							// draw desired target corners in the current tracking image
							cv::line(I_track, cv::Point(objectDetection.object_corners[0].x,objectDetection.object_corners[0].y), cv::Point(objectDetection.object_corners[1].x,objectDetection.object_corners[1].y), cv::Scalar(255,0,0), 1, 8, 0 );
							cv::line(I_track, cv::Point(objectDetection.object_corners[1].x,objectDetection.object_corners[1].y), cv::Point(objectDetection.object_corners[2].x,objectDetection.object_corners[2].y), cv::Scalar(255,0,0), 1, 8, 0 );
							cv::line(I_track, cv::Point(objectDetection.object_corners[2].x,objectDetection.object_corners[2].y), cv::Point(objectDetection.object_corners[3].x,objectDetection.object_corners[3].y), cv::Scalar(255,0,0), 1, 8, 0 );
							cv::line(I_track, cv::Point(objectDetection.object_corners[3].x,objectDetection.object_corners[3].y), cv::Point(objectDetection.object_corners[0].x,objectDetection.object_corners[0].y), cv::Scalar(255,0,0), 1, 8, 0 );
							// show the first frame
							// cv::imshow( "I_tracking",I_track );
							// cvWaitKey();

							/**
							 *  record the current target corners in the image plane
							 */
							objectDetection.scene_corners_current.clear();
							for( int i = 0; i < objectDetection.object_corners.size(); i++ )
							{
								objectDetection.scene_corners_current.push_back(objectDetection.object_corners[i]);
							}

							/**
							 *  initial visual servoing control
							 */

						}
						continue;
					}	// if ( motionState == MotionState::TARGET_DETECTION )

					/**
					 * 	4. tracking
					 * 	tracking : planar object tracking using ESM
					 */
					if ( robot._motionState.motionStateCurrent == MotionState::TARGET_TRACKING )
					{
						/*
						 *  convert the image format
						 */
//						IplImage I_Ipl;
//						I_Ipl = M1.operator _IplImage();
//						IplImage2imageStruct ( &I_Ipl, &I);
						Mat2imageStruct ( M1, &I);

						/*
						 *	Perform the tracking
						 */
						if (MakeTrack(&esmTrack, I))
							return (1);

						/**
						 * current object corners
						 */
						// in pixel
						std::vector<cv::Point2f> scene_corners;
						Homotransform ( esmTrack.homog, objectDetection.scene_corners_initial, objectDetection.scene_corners_current );
						// in meter
						PlanarObjectDetection::pixeltometer(cameraMatrix_1_rectified, objectDetection.scene_corners_current, objectDetection.scene_corners_current_meter );

						/*
						 *  show the results
						 */
						// draw scene corners
						I_track = M1_color.clone();
						template_mark( I_track, objectDetection.scene_corners_current );	// draw the four corners of the target

						// put the reprojection of the current pattern in the current image
						imageStruct* pattern_c = GetPatc (&esmTrack);
						// put to color image
						int x_pattern_dsp = 5;
						int y_pattern_dsp = 5;
//						I_Ipl = I_track.operator _IplImage();
//						for(int i = 0; i< pattern_c->rows; i++)
//						{
//							for(int j = 0; j< pattern_c->cols; j++)
//							{
//								int ii = i + y_pattern_dsp;
//								int jj = j + x_pattern_dsp;
//								(I_Ipl.imageData + I_Ipl.widthStep*ii)[jj*I_Ipl.nChannels]   = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
//								(I_Ipl.imageData + I_Ipl.widthStep*ii)[jj*I_Ipl.nChannels+1] = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
//								(I_Ipl.imageData + I_Ipl.widthStep*ii)[jj*I_Ipl.nChannels+2] = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
//							}
//						}
						for(int i = 0; i< pattern_c->rows; i++)
						{
							for(int j = 0; j< pattern_c->cols; j++)
							{
								int ii = i + y_pattern_dsp;
								int jj = j + x_pattern_dsp;
								I_track.at<cv::Vec3b>(ii,jj)[0]  = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
								I_track.at<cv::Vec3b>(ii,jj)[1]  = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
								I_track.at<cv::Vec3b>(ii,jj)[2]  = (uchar)*(pattern_c->data + i*pattern_c->cols+j);
							}
						}
						// draw a white square around tracking pattern
						cv::rectangle(I_track, cv::Rect(x_pattern_dsp-1,y_pattern_dsp-1, pattern_c->cols,pattern_c->rows), cv::Scalar(255,255,255), 2, 8, 0);
						// draw desired target corners in the current tracking image
						cv::line(I_track, cv::Point(objectDetection.object_corners[0].x,objectDetection.object_corners[0].y), cv::Point(objectDetection.object_corners[1].x,objectDetection.object_corners[1].y), cv::Scalar(255,0,0), 1, 8, 0 );
						cv::line(I_track, cv::Point(objectDetection.object_corners[1].x,objectDetection.object_corners[1].y), cv::Point(objectDetection.object_corners[2].x,objectDetection.object_corners[2].y), cv::Scalar(255,0,0), 1, 8, 0 );
						cv::line(I_track, cv::Point(objectDetection.object_corners[2].x,objectDetection.object_corners[2].y), cv::Point(objectDetection.object_corners[3].x,objectDetection.object_corners[3].y), cv::Scalar(255,0,0), 1, 8, 0 );
						cv::line(I_track, cv::Point(objectDetection.object_corners[3].x,objectDetection.object_corners[3].y), cv::Point(objectDetection.object_corners[0].x,objectDetection.object_corners[0].y), cv::Scalar(255,0,0), 1, 8, 0 );
						// show results
						// cv::imshow( "I_tracking",  I_track);

						/**
						 *  detect the target in the right image plane
						 */
						int		maxCorners		= 100;
						double	qualityLevel	= 0.001;
						double	minDistance		= 3;
						int		blockSize		= 3;		// (non-maximum suppression?) Size of the averaging block for
															//	computing derivative covariation matrix over each pixel neighborhood
						int		use_harris		= false;
						double	k				= 0.04;


//						int x 		= objectDetection.object_corners[0].x;
//						int y 		= objectDetection.object_corners[0].y;
//						int width 	= objectDetection.object_corners[1].x - objectDetection.object_corners[0].x;
//						int height 	= objectDetection.object_corners[3].y - objectDetection.object_corners[0].y;
						int x 		= objectDetection.scene_corners_current[0].x;
						int y 		= objectDetection.scene_corners_current[0].y;
						int width 	= objectDetection.scene_corners_current[1].x - objectDetection.scene_corners_current[0].x;
						int height 	= objectDetection.scene_corners_current[3].y - objectDetection.scene_corners_current[0].y;
						cv::Mat M1_roi(M1,cv::Rect(x,y,width,height));
						/// Apply corner detection
						std::vector<cv::Point2f> corners_left;
						cv::goodFeaturesToTrack( M1_roi, corners_left, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, use_harris);

						/// Set the neeed parameters to find the refined corners
						cv::Size winSize = cv::Size( 5, 5 );
						cv::Size zeroZone = cv::Size( -1, -1 );
						cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );

						/// Calculate the refined corner locations
						cv::cornerSubPix( M1_roi, corners_left, winSize, zeroZone, criteria );

						/// M1_roi coordinates -> M1 coordinates
						for(int i=0; i < corners_left.size(); i++)
						{
							corners_left[i].x += x;
							corners_left[i].y += y;
							// cv::circle( M1_color, cv::Point(corners_left[i].x, corners_left[i].y), 2, cv::Scalar(0,255,0), -1 );
						}

						// track the detected feature in the right frame
						std::vector<cv::Point2f> corners_right;
						std::vector<unsigned char> status_LK;
						std::vector<float> err_LK;
						cv::Size winSize_LK(50,5);
						int maxLevel = 1;
						cv::TermCriteria criteria_LK = cv::TermCriteria( cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
						double derivLambda=0.5;
						int flags = cv::OPTFLOW_USE_INITIAL_FLOW;
						for(int i = 0; i < corners_left.size(); i++)
							corners_right.push_back(cv::Point2f(0,0));
						double minEigThreshold = 0.0001;
						corners_right.reserve(corners_left.size());
//						cv::calcOpticalFlowPyrLK(  I1, I2, corners1, corners2, status, err, winSize_LK, maxLevel,
//														criteria_LK, derivLambda,flags, minEigThreshold);
						cv::calcOpticalFlowPyrLK(  M1, M2, corners_left, corners_right, status_LK, err_LK);		// perform good with default parameter

						// select the tracked
						// draw correspondences
						std::vector<cv::Point2f> corners_leftMatched, corners_rightMatched;
						for(int i=0; i < corners_left.size(); i++)
						{
							if(status_LK[i] == 1)
							{
								corners_leftMatched.push_back(corners_left[i]);
								corners_rightMatched.push_back(corners_right[i]);
								//cv::circle( M1_color, cv::Point(corners_left[i].x, corners_left[i].y), 2, cv::Scalar(0,255,0), -1 );
								//cv::circle( M2_color, cv::Point(corners_right[i].x, corners_right[i].y), 2, cv::Scalar(0,255,0), -1 );
							}
						}

						// homography
						cv::Mat H_LK;
						std::vector<unsigned char> mask_LK;
						H_LK = cv::findHomography( corners_leftMatched, corners_rightMatched, mask_LK, cv::RANSAC, 1);

						// transform the target corners
						cv::perspectiveTransform(objectDetection.scene_corners_current, objectDetection.scene_corners_currentRight, H_LK);

						//-- Draw lines between the corners (the mapped object in the scene - image_2 )
						cv::line( M2_color, objectDetection.scene_corners_currentRight[0], objectDetection.scene_corners_currentRight[1], cv::Scalar(0, 255, 0), 1 );
						cv::line( M2_color, objectDetection.scene_corners_currentRight[1], objectDetection.scene_corners_currentRight[2], cv::Scalar( 0, 255, 0), 1 );
						cv::line( M2_color, objectDetection.scene_corners_currentRight[2], objectDetection.scene_corners_currentRight[3], cv::Scalar( 0, 255, 0), 1 );
						cv::line( M2_color, objectDetection.scene_corners_currentRight[3], objectDetection.scene_corners_currentRight[0], cv::Scalar( 0, 255, 0), 1 );

						/*
						 *  visual servoing control
						 */
//						// error and interaction matrix
//						cv::Mat err(2*objectDetection.object_corners.size(), 1, CV_64FC1);
//						cv::Mat Lsv = cv::Mat::zeros(2*objectDetection.object_corners.size(), 1, CV_64FC1);
//						cv::Mat Lsw = cv::Mat::zeros(2*objectDetection.object_corners.size(), 1, CV_64FC1);
//						for(int i = 0; i < objectDetection.scene_corners_current_meter.size(); i++)
//						{
//							err.at<double>(2*i,0) = objectDetection.scene_corners_current_meter[i].x - objectDetection.object_corners_meter[i].x;
//							err.at<double>(2*i+1,0) = objectDetection.scene_corners_current_meter[i].y - objectDetection.object_corners_meter[i].y;
//
//							double x = objectDetection.scene_corners_current_meter[i].x;
//							double y = objectDetection.scene_corners_current_meter[i].y;
//							double Z = Z_IJPU;
//							Lsv.at<double>(2*i,0) = x/Z;
//							Lsv.at<double>(2*i+1,0) = y/Z;
//							Lsw.at<double>(2*i,0) = 1 + x*x;
//							Lsw.at<double>(2*i+1,0) = x*y;
//						}
//
//						// constant linear velocity
//						cv::Mat temp(1, 1, CV_64FC1);
//						temp = -Lsw.inv(cv::DECOMP_SVD)*(lambd*err+vl_vs*Lsv);
//						va_vs = temp.at<double>(0,0);
//
//						expectedVl = vl_vs;
//						expectedVa = -va_vs;
//
						visualServoing.visualServoing_IJPU( objectDetection.object_corners_meter, objectDetection.scene_corners_current_meter,
															visualServoing._Z_IJPU, visualServoing._lambd_IJPU, vl_vs, va_vs );

						expectedVl = vl_vs;
						expectedVa = -va_vs;	// for Lina

						/**
						 *	decide whether have to do obstacle avoidance
						 *	collision detection
						 */
//						bool b_collision = false;
//						int indexClusters_reactive;
//						double s_path = 0;
//						double preavoidDistance = 1.5;
//						control.collisioinPredictionVisualServoing(las,navi,laserSensing.Clusters,robot,expectedVl,expectedVa, preavoidDistance, b_collision,indexClusters_reactive,s_path);

//						bool b_collision = false;
//						int indexClusters_reactive;
//						double s_path = 0;
//						double preavoidDistance = 1.0;
//						control.collisioinPredictionStraightLineFollowing( las, navi, laserSensing.Clusters,odometry,robot,
//																		   expectedVl,expectedVa, control._preavoidDistance, control._b_collision,
//																		   control._indexClusters_reactive, control._s_path, control._trajectoryPrediction);
//						control.collisioinDetectionConstantControl( las, navi, laserSensing.Clusters, robot, expectedVl, expectedVa,preavoidDistance,
//																	b_collision, indexClusters_reactive, s_path, control._trajectoryPrediction);

						control.collisioinPredictionGeneral( las, navi, laserSensing.Clusters, odometry, robot, expectedVl, -expectedVa,	// attention there is a min for Lina
															 control._preavoidDistance, control._predictionMethod.method, control._b_collision,
															 control._indexClusters_reactive, control._s_path, control._trajectoryPrediction);
						// remember the reative obstacle
						laserSensing.previousReactiveObstacleUpdate( las, control._indexClusters_reactive, laserSensing.ConvexChains, Odometries[loopNumber],
																	 laserSensing._obstaclePrevious, laserSensing._odometryPrevous);

						//std::cout << " prediction :	" <<  control._s_path << "	" << robot._radius + robot._disFollowing << "	" << control._b_collision << std::endl;

						// move to the end
//						I_laserReactiveObstacle = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//						navi.drawLaserCoordinate(width_area, height_area, I_laserReactiveObstacle);
//						myDraw.drawPredictedTrajecotry(control._trajectoryPrediction,myDraw._wScale_meter2laserwindow,myDraw._hScale_meter2laserwindow, (float)robot._radius, I_laserReactiveObstacle);
//						navi.drawObstacleReactive(las, width_area, height_area, laserSensing.Clusters, laserSensing.ConvexChains, control._b_collision, control._indexClusters_reactive, I_laserReactiveObstacle);

						if ( control._b_collision == true )
						{
							// change the state
							robot._motionState.motionStateCurrent = MotionState::OBSTACLE_AVOIDANCE;

							/**
							 *  remember the target planar parameters in the current camera frame
							 *
							 */
							///  triangular the target points into 3-D space using stereo vision
							//						cv::Mat points_4D( 4, objectDetection.scene_corners_current.size(), CV_32F);
							//						cv::triangulatePoints( P1, P2, objectDetection.scene_corners_current,
							//												objectDetection.scene_corners_currentRight, points_4D );

							// transform to old version
							double cam1Ext_a[12] = { P1.at<double>(0,0), P1.at<double>(0,1), P1.at<double>(0,2), P1.at<double>(0,3),
													 P1.at<double>(1,0), P1.at<double>(1,1), P1.at<double>(1,2), P1.at<double>(1,3),
													 P1.at<double>(2,0), P1.at<double>(2,1), P1.at<double>(2,2), P1.at<double>(2,3) };
							CvMat cam1Ext = cvMat(3,4,CV_64F,cam1Ext_a);
							double cam2Ext_a[12] = { P2.at<double>(0,0), P2.at<double>(0,1), P2.at<double>(0,2), P2.at<double>(0,3),
													 P2.at<double>(1,0), P2.at<double>(1,1), P2.at<double>(1,2), P2.at<double>(1,3),
													 P2.at<double>(2,0), P2.at<double>(2,1), P2.at<double>(2,2), P2.at<double>(2,3) };
							CvMat cam2Ext = cvMat(3,4,CV_64F,cam2Ext_a);
							std::cout << "P1 \n" << P1 << std::endl;
							std::cout << "P2 \n" << P2 << std::endl;

							CvMat *_pointImg1 = cvCreateMat(2,4,CV_64FC1);
							CvMat *_pointImg2 = cvCreateMat(2,4,CV_64FC1);
							for(int i = 0; i < 4; i++ )
							{
								CV_MAT_ELEM( *_pointImg1, double, 0, i ) = objectDetection.scene_corners_current[i].x;
								CV_MAT_ELEM( *_pointImg1, double, 1, i ) = objectDetection.scene_corners_current[i].y;
								CV_MAT_ELEM( *_pointImg2, double, 0, i ) = objectDetection.scene_corners_currentRight[i].x;
								CV_MAT_ELEM( *_pointImg2, double, 1, i ) = objectDetection.scene_corners_currentRight[i].y;
							}

							CvMat *point3D = cvCreateMat(4,4,CV_64F) ;
							cvTriangulatePoints(&cam1Ext, &cam2Ext, _pointImg1, _pointImg2, point3D);

							// normalize
							for(int i = 0; i < 4; i++ )
							{
								CV_MAT_ELEM( *point3D, double, 0, i ) = CV_MAT_ELEM( *point3D, double, 0, i )/CV_MAT_ELEM( *point3D, double, 3, i );
								CV_MAT_ELEM( *point3D, double, 1, i ) = CV_MAT_ELEM( *point3D, double, 1, i )/CV_MAT_ELEM( *point3D, double, 3, i );
								CV_MAT_ELEM( *point3D, double, 2, i ) = CV_MAT_ELEM( *point3D, double, 2, i )/CV_MAT_ELEM( *point3D, double, 3, i );
								std::cout <<  CV_MAT_ELEM( *point3D, double, 0, i ) << "	" << CV_MAT_ELEM( *point3D, double, 1, i ) << "		"
									<< CV_MAT_ELEM( *point3D, double, 2, i ) << std::endl;

							}

							// without transform
//							/// remember the plane method one: normal and perpendicular distance
//							// vector one v1 = p1 - p0
//							double a = CV_MAT_ELEM( *point3D, double, 0, 1 ) - CV_MAT_ELEM( *point3D, double, 0, 0 );
//							double b = CV_MAT_ELEM( *point3D, double, 1, 1 ) - CV_MAT_ELEM( *point3D, double, 1, 0 );
//							double c = CV_MAT_ELEM( *point3D, double, 2, 1 ) - CV_MAT_ELEM( *point3D, double, 2, 0 );
//							cv::Mat v1 =(cv::Mat_<double>(3,1) << a, b, c);
//							// vector two v2 = p3 - p0
//							a = CV_MAT_ELEM( *point3D, double, 0, 3 ) - CV_MAT_ELEM( *point3D, double, 0, 0 );
//							b = CV_MAT_ELEM( *point3D, double, 1, 3 ) - CV_MAT_ELEM( *point3D, double, 1, 0 );
//							c = CV_MAT_ELEM( *point3D, double, 2, 3 ) - CV_MAT_ELEM( *point3D, double, 2, 0 );
//							cv::Mat v2 =(cv::Mat_<double>(3,1) << a, b, c);
//
//							normalRemebered = v1.cross(v2);
//							double scale = std::sqrt(	normalRemebered.at<double>(0,0)*normalRemebered.at<double>(0,0)
//														+ normalRemebered.at<double>(1,0)*normalRemebered.at<double>(1,0)
//														+ normalRemebered.at<double>(2,0)*normalRemebered.at<double>(2,0));
//							normalRemebered = normalRemebered / scale;
//							// projection distance
//							a = CV_MAT_ELEM( *point3D, double, 0, 0 );
//							b = CV_MAT_ELEM( *point3D, double, 1, 0 );
//							c = CV_MAT_ELEM( *point3D, double, 2, 0 );
//							cv::Mat v3 =(cv::Mat_<double>(3,1) << a, b, c);
//							perpendicularDistance = v3.dot(normalRemebered);
//
//							/// remember the plane method two: three points
//							//  P 				Q
//							//
//							//
//							//	R
//							vpPoint P_plane, Q_plane, R_plane;
//							P_plane.set_X(CV_MAT_ELEM( *point3D, double, 0, 0 ));
//							P_plane.set_Y(CV_MAT_ELEM( *point3D, double, 1, 0 ));
//							P_plane.set_Z(CV_MAT_ELEM( *point3D, double, 2, 0 ));
//							Q_plane.set_X(CV_MAT_ELEM( *point3D, double, 0, 1 ));
//							Q_plane.set_Y(CV_MAT_ELEM( *point3D, double, 1, 1 ));
//							Q_plane.set_Z(CV_MAT_ELEM( *point3D, double, 2, 1 ));
//							R_plane.set_X(CV_MAT_ELEM( *point3D, double, 0, 3 ));
//							R_plane.set_Y(CV_MAT_ELEM( *point3D, double, 1, 3 ));
//							R_plane.set_Z(CV_MAT_ELEM( *point3D, double, 2, 3 ));
//							lost_Plane.init( P_plane, Q_plane, R_plane );

							/*
							 *  Transformation the points from the right camera frame to the left camera frame
							 */
							cv::Mat l_T_r = (cv::Mat_<double>(4,4) << 1, 0, 0, base,
																	  0, 1, 0,    0,
																	  0, 0,	1,	  0,
																	  0, 0,	0,	  1);
							double pts_l[3][4];
							for( int i = 0; i < 4; i++ )
							{
								cv::Mat pt_r = (cv::Mat_<double>(4,1) <<	CV_MAT_ELEM( *point3D, double, 0, i ),
																			CV_MAT_ELEM( *point3D, double, 1, i ),
																			CV_MAT_ELEM( *point3D, double, 2, i ),
																											  1 );
								cv::Mat pt_l =  l_T_r*pt_r;
								pt_l =  pt_l/ pt_l.at<double>(3,0);
								pts_l[0][i] = pt_l.at<double>(0,0);
								pts_l[1][i] = pt_l.at<double>(1,0);
								pts_l[2][i] = pt_l.at<double>(2,0);
							}
							cv::Mat pts_left(3, 4, CV_64F, pts_l);

							/// remember the plane method one: normal and perpendicular distance
							// vector one v1 = p1 - p0
							double a = pts_left.at<double>(0,1)  - pts_left.at<double>(0,0);
							double b = pts_left.at<double>(1,1)  - pts_left.at<double>(1,0);
							double c = pts_left.at<double>(2,1)  - pts_left.at<double>(2,0);
							cv::Mat v1 =(cv::Mat_<double>(3,1) << a, b, c);
							// vector two v2 = p3 - p0
							a = pts_left.at<double>(0,3)  - pts_left.at<double>(0,0);
							b = pts_left.at<double>(1,3)  - pts_left.at<double>(1,0);
							c = pts_left.at<double>(2,3)  - pts_left.at<double>(2,0);
							cv::Mat v2 =(cv::Mat_<double>(3,1) << a, b, c);

							normalRemebered = v1.cross(v2);
							double scale = std::sqrt(	normalRemebered.at<double>(0,0)*normalRemebered.at<double>(0,0)
														+ normalRemebered.at<double>(1,0)*normalRemebered.at<double>(1,0)
														+ normalRemebered.at<double>(2,0)*normalRemebered.at<double>(2,0));
							normalRemebered = normalRemebered / scale;
							// projection distance using p0
							a = pts_left.at<double>(0,0);
							b = pts_left.at<double>(1,0);
							c = pts_left.at<double>(2,0);
							cv::Mat v3 =(cv::Mat_<double>(3,1) << a, b, c);
							perpendicularDistance = v3.dot(normalRemebered);

							/// remember the plane method two: three points
							//  P 				Q
							//
							//
							//	R
							vpPoint P_plane, Q_plane, R_plane;
							P_plane.set_X( pts_left.at<double>(0,0) );
							P_plane.set_Y( pts_left.at<double>(1,0) );
							P_plane.set_Z( pts_left.at<double>(2,0) );
							Q_plane.set_X( pts_left.at<double>(0,1) );
							Q_plane.set_Y( pts_left.at<double>(1,1) );
							Q_plane.set_Z( pts_left.at<double>(2,1) );
							R_plane.set_X( pts_left.at<double>(0,3) );
							R_plane.set_Y( pts_left.at<double>(1,3) );
							R_plane.set_Z( pts_left.at<double>(2,3) );
							lost_Plane.init( P_plane, Q_plane, R_plane );

							/// remember the four target corners in the image plane
							scene_corners_remembered.clear();
							scene_corners_remembered = objectDetection.scene_corners_current;
							objectDetection.scene_corners_lost.clear();
							objectDetection.scene_corners_lost = objectDetection.scene_corners_current;

							/**
							 *  remember the current camera frame with respect to the world frame
							 */
							w_T_lost = w_T_c;

							// release tracking
							if (esmTrack.images != NULL)
							{
								FreeTrack(&esmTrack);
								esmTrack.images = NULL;
								esmTrack.trackdata = NULL;
							}
#ifdef Generate_Log
							/*
							 *  restore the object 3D points in current camera frame
							 */
							lTargetInCamera << CV_MAT_ELEM( *point3D, double, 0, 0 ) << "\t" << CV_MAT_ELEM( *point3D, double, 1, 0 ) << "\t" << CV_MAT_ELEM( *point3D, double, 2, 0 ) << "\t"
									<< CV_MAT_ELEM( *point3D, double, 0, 1 ) << "\t" << CV_MAT_ELEM( *point3D, double, 1, 1 ) << "\t" << CV_MAT_ELEM( *point3D, double, 2, 1 ) << "\t"
									<< CV_MAT_ELEM( *point3D, double, 0, 2 ) << "\t" << CV_MAT_ELEM( *point3D, double, 1, 2 ) << "\t" << CV_MAT_ELEM( *point3D, double, 2, 2 ) << "\t"
									<< CV_MAT_ELEM( *point3D, double, 0, 3 ) << "\t" << CV_MAT_ELEM( *point3D, double, 1, 3 ) << "\t" << CV_MAT_ELEM( *point3D, double, 2, 3 ) << std::endl;

#endif
						}

					}	// if ( motionState == MotionState::TARGET_TRACKING )

					/**
					 * 	stop
					 */
					if ( robot._motionState.motionStateCurrent == MotionState::STOP )
					{
						expectedVl = 0;
						expectedVa = 0;
						if ( _newVl < 0.001 && _newVa < PI/150 )
						{
							stop_number++;
							if( stop_number > stop_threshold )
							{
//								b_rotateFinding = true;
//								b_rotateFindingStart = true;
								robot._motionState.b_rotateFinding = true;
								robot._motionState.b_rotateFindingStart = true;
								robot._motionState.motionStateCurrent = robot._motionState.motionStateRemembered;
								//motionState = MotionState::TARGET_FINDING;
								stop_number = 0;
							}
						}

						if ( robot._motionState.motionStateRemembered ==  MotionState::TARGET_FINDING )
						{
							/**
							 *  estimate the feautres
							 */
							// calculate the transformation of the current finding camera frame wrt the lost frame
							vpHomogeneousMatrix lost_T_find;
							lost_T_find = w_T_lost.inverse()*w_T_c;

							vpHomogeneousMatrix find_T_lost;
							find_T_lost = lost_T_find.inverse();

							// estimate the visual feature in the current image frame
							objectDetection.visualFeatureEstimation(find_T_lost, lost_Plane, cameraMatrix_1_rectified, objectDetection);

//							// draw the estimatd features
//							navi.drawFeatureEstimation(objectDetection.estimated_corners_current, robot._motionState.motionStateCurrent, M1_color, I_track, I_featureEstimation);

							/**
							 *  record the current target corners in the image plane
							 */
							objectDetection.scene_corners_current.clear();
							for( int i = 0; i < objectDetection.object_corners.size(); i++ )
							{
								objectDetection.scene_corners_current.push_back(objectDetection.estimated_corners_current[i]);
							}
						}

					}

					/**
					 * 	target finding
					 */
					if ( robot._motionState.motionStateCurrent == MotionState::TARGET_FINDING )
					{
						if ( robot._motionState.motionStateRemembered  == MotionState::TARGET_DETECTION )	// directly rotate and find
						{
							/**
							 *  2. rotate and find: approach one, use field of view
							 */
							if ( robot._motionState.b_rotateFinding == true )
							{
								/**
								 *   2.1 set the initial value
								 */
								if ( robot._motionState.b_rotateFindingStart == true )
								{
									thetaFinding_ini = odometry.phi;
									dthetaFinding = PI/6;
									thetaFinding_threshold = PI/30 ;
									thetaFinding_va = -PI/30;			// left rotate
									robot._motionState.b_rotateFindingStart = false;
								}	// if ( b_rotateFinding == true )
								/**
								 *   2.2 rotate and find
								 */
								else	// b_rotateFindingStart = ture;
								{
									double error = std::abs( std::abs(odometry.phi - thetaFinding_ini) -dthetaFinding );
									std::cout << "std::abs(odometry.phi - thetaFinding_ini) : " << std::abs(odometry.phi - thetaFinding_ini)*180/PI << std::endl;
									std::cout << "angular error for target finding : " << error*180/PI << std::endl;
									std::cout << "angular threshold for target finding : " << thetaFinding_threshold*180/PI << std::endl;
									if( error < thetaFinding_threshold )
									{
										robot._motionState.b_rotateFinding 			= false;
										//robot._motionState.b_targetFindingDetection = true;
										robot._motionState.motionStateCurrent    	= MotionState::STOP;
										robot._motionState.motionStateRemembered 	= MotionState::TARGET_DETECTION;
									}
									else
									{
										expectedVa = thetaFinding_va;
										std::cout << "expectedVa finding : " << expectedVa*180/PI << std::endl;
									}
								}
							}	// if ( b_rotateFinding == true )
						}

						/**
						 *  target finding approach three: reconver the homography matrix
						 *  align the gravity center of the target in the center of the current image plane
						 */
						if( robot._motionState.motionStateRemembered  == MotionState::TARGET_FINDING &&
								robot._motionState.b_rotateFinding == true )
						{
//							// calculate the transformation of the current finding camera frame wrt the lost frame
//							vpHomogeneousMatrix lost_T_find;
//							lost_T_find = w_T_lost.inverse()*w_T_c;
//
//							vpHomogeneousMatrix find_T_lost;
//							find_T_lost = lost_T_find.inverse();
//
//							/// recover the homography matrix
//							vpHomography find_H_lost( find_T_lost, lost_Plane );
//
//							vpCameraParameters cam(cameraMatrix_1_rectified.at<double>(0,0), cameraMatrix_1_rectified.at<double>(1,1),
//                                                   cameraMatrix_1_rectified.at<double>(0,2), cameraMatrix_1_rectified.at<double>(1,2));
//
//
//							// forward project the target points onto the current finding image plane in meter
//							std::vector<cv::Point2f> lost_pts;
//							for( int i = 0; i < 4; i++ )
//							{
//								double u, v;
//								u = objectDetection.scene_corners_lost[i].x;
//								v = objectDetection.scene_corners_lost[i].y;
//								double x, y;
//								vpPixelMeterConversion::convertPointWithoutDistortion( cam, u, v, x, y );
//								lost_pts.push_back(cv::Point2f(x,y));
//							}
//
//							cv::Mat f_H_l = (cv::Mat_<double>(3,3) << find_H_lost[0][0], find_H_lost[0][1], find_H_lost[0][2],
//																	  find_H_lost[1][0], find_H_lost[1][1], find_H_lost[1][2],
//																	  find_H_lost[2][0], find_H_lost[2][1], find_H_lost[2][2]);
//							std::vector<cv::Point2f> find_pts;
//							cv::perspectiveTransform( lost_pts, find_pts, f_H_l);
//
//							// calculate the gravity center
//							double x_g = 0;
//							double y_g = 0;
//							for ( int i = 0; i < 4; i++ )
//							{
//								x_g = x_g + find_pts[i].x ;
//								y_g = y_g + find_pts[i].y;
//							}
//							x_g = x_g/4;
//							y_g = y_g/4;
//
//							// gravity center conversion to pixel coordinates
//							//vpImagePoint pts_Image[4];
//
//
//							double u_g, v_g;
//							vpMeterPixelConversion::convertPointWithDistortion(cam, x_g, y_g, u_g, v_g);


							// calculate the transformation of the current finding camera frame wrt the lost frame
							vpHomogeneousMatrix lost_T_find;
							lost_T_find = w_T_lost.inverse()*w_T_c;

							vpHomogeneousMatrix find_T_lost;
							find_T_lost = lost_T_find.inverse();

							// estimate the visual feature in the current image frame
							objectDetection.visualFeatureEstimation(find_T_lost, lost_Plane, cameraMatrix_1_rectified, objectDetection);

//							// draw the estimated features
//							navi.drawFeatureEstimation(objectDetection.estimated_corners_current, robot._motionState.motionStateCurrent, M1_color, I_track, I_featureEstimation);

							/**
							 *  record the current target corners in the image plane
							 */
							objectDetection.scene_corners_current.clear();
							for( int i = 0; i < objectDetection.object_corners.size(); i++ )
							{
								objectDetection.scene_corners_current.push_back(objectDetection.estimated_corners_current[i]);
							}

							double alignment_threshold = 10;
							double error_pixel =  objectDetection.pt_gravityCenter_current.x - M1.cols/2;	// error along u-axis
							if( std::abs(error_pixel) < alignment_threshold )
							{
								robot._motionState.b_rotateFinding = false;
								robot._motionState.b_targetFindingDetection = true;
//								b_rotateFinding = false;
//								b_targetFindingDetection = true;
								robot._motionState.motionStateCurrent    = MotionState::STOP;
								robot._motionState.motionStateRemembered = MotionState::TARGET_DETECTION;
							}
							else
							{
								// visual servoing control to alignment

								// using fixed angular velocity
//								thetaFinding_va = PI/30;
//								expectedVa = error_pixel > 0 ? thetaFinding_va : -thetaFinding_va;

								// using rotation heading controllor (RH)
								double lambda = 0.2;	// 0.1
								double dtheta = std::atan(objectDetection.pt_gravityCenter_current_meter.x);
								expectedVa = lambda*dtheta;
								std::cout << "expectedVa finding : " << expectedVa*180/PI << std::endl;
							}
							std::cout << "target finding: " << objectDetection.pt_gravityCenter_current.x << "	"  << objectDetection.pt_gravityCenter_current_meter.x << "		"<< std::abs(error_pixel) << std::endl;
						}		// if( b_rotateFinding == true )
					}		// if ( motionState == MotionState::TARGET_FINDING )

					/**
					 * 	obstacle avoidance
					 */
					if ( robot._motionState.motionStateCurrent == MotionState::OBSTACLE_AVOIDANCE )
					{
						/*
						 *  estimate the current visual features
						 */
						// calculate the transformation of the current finding camera frame wrt the lost frame
						vpHomogeneousMatrix lost_T_find;
						lost_T_find = w_T_lost.inverse()*w_T_c;

						vpHomogeneousMatrix find_T_lost;
						find_T_lost = lost_T_find.inverse();

						// estimate the visual feature in the current image frame
						objectDetection.visualFeatureEstimation(find_T_lost, lost_Plane, cameraMatrix_1_rectified, objectDetection);

						/**
						 *  updata the current estimated target corners in the image plane
						 */
						objectDetection.scene_corners_current.clear();
						objectDetection.scene_corners_current_meter.clear();
						for( int i = 0; i < objectDetection.object_corners.size(); i++ )
						{
							objectDetection.scene_corners_current.push_back(objectDetection.estimated_corners_current[i]);
							objectDetection.scene_corners_current_meter.push_back( objectDetection.estimated_corners_current_meter[i] );
						}

						/*
						 *  2.1	using predictor to decide the potential collision distance
						 */
//						bool b_collision = false;
//						int indexClusters_reactive;
//						double s_path = 0;
//						double preavoidDistance = 1.0;
						//control._b_collision = false;
//						control.collisioinPredictionStraightLineFollowing( las, navi, laserSensing.Clusters, odometry, robot, _newVl,
//																		   _newVa, control._preavoidDistance, control._b_collision,
//																		   control._indexClusters_reactive, control._s_path );

//						control.collisioinPredictionStraightLineFollowing( las, navi, laserSensing.Clusters,odometry,robot,
//																		   expectedVl,expectedVa, control._preavoidDistance, control._b_collision,
//																		   control._indexClusters_reactive, control._s_path, control._trajectoryPrediction);

//						control.collisioinPredictionGeneral( las, navi, laserSensing.Clusters, odometry, robot, expectedVl, -expectedVa,		// attention there is a min for Lina
//																control._preavoidDistance, control._predictionMethod.method, control._b_collision,
//																control._indexClusters_reactive, control._s_path, control._trajectoryPrediction);

//						float Va_1, Vl_1;	// control law for the first task
//						Vl_1 = expectedVl;
//						visualServoing.visualServoing_IJPU( objectDetection.object_corners_meter, objectDetection.scene_corners_current_meter,
//																											visualServoing._Z_IJPU, visualServoing._lambd_IJPU, Vl_1, Va_1);
//						// for Lina
//						Va_1 = -Va_1;
//						// prediction
//						control.collisioinDetectionConstantControl( las, navi, laserSensing.Clusters, robot, Vl_1,  Va_1, preavoidDistance,
//																	b_collision, indexClusters_reactive, s_path, control._trajectoryPrediction);

						/*
						 * 	2.2 linear velocity
						 */

						/*
						 * 	2.2.1 linear velocity based on potential collision distance
						 */
						//bool b_escape = true;	// assume that there is no obstacle to be reactived
						// control._b_leave = true; 	// assume that there is no obstacle to be reactived
						double vl_expected;
						control.linearVelocityBasedOnPotentialCollisionDistance(control._b_collision, robot, control._s_path, vl_expected, control._b_leave );
						expectedVl = vl_expected;

						if ( control._b_collision == true && expectedVl > robot._vMaxOA )
							expectedVl = robot._vMaxOA;
						if ( expectedVl > robot._vMax )
							expectedVl = robot._vMax;

						expectedVl = robot._vMax;

						/*
						 *  2.3	angular velocity
						 */

						/*
						 * 	2.3.1 task one: straight line following
						 * 	notice that distance is in millimiter, delta for control is in meter
						 */
//						double Va_1, Vl_1;	// control law for the first task
//						Vl_1 = expectedVl;
//						control.straightlineFollowing(odometry, Vl_1, Va_1);
//						Va_1 = - Va_1;

						/*
						 * 	2.3.1 task one: virtual visual servoing control for goal task
						 * 	notice that distance is in millimiter, delta for control is in meter
						 */

						float Va_1, Vl_1;	// control law for the first task
						Vl_1 = expectedVl;
						visualServoing.visualServoing_IJPU( objectDetection.object_corners_meter, objectDetection.estimated_corners_current_meter,
															visualServoing._Z_IJPU, visualServoing._lambd_IJPU, Vl_1, Va_1);
						Va_1 = - Va_1;	// for Lina
						// prediction and identify the reactive obstacle
						control.collisioinPredictionGeneral( las, navi, laserSensing.Clusters, odometry, robot, Vl_1, -Va_1,		// attention there is a min for Lina
															 control._preavoidDistance, control._predictionMethod.method, control._b_collision,
															 control._indexClusters_reactive, control._s_path, control._trajectoryPrediction);

						/*
						 * 	2.3.2 task two: obstacle avoidance controller
						 * 	Notice that the path has directions, therefore there are signs for delta
						 * 	output Va_2, Vl_2
						 */
						double Va_2, Vl_2;	// control law for the second task
						Vl_2 = expectedVl;
						double dis_robotTopath;	// from robot reference point to the following path

						// if ( control._b_collision == true && control._s_path <= robot._disReactive + robot._radius )	// do reactive
						// if ( control._b_collision == true && control._s_path <= robot._disReactive + robot._radius )	// do reactive
						if ( control._b_collision == true )	// do reactive
						{
							/*
							 * 	2.3.2.1 fitting the boundary of the reactive obstacle by Bspline
							 */
							std::vector<std::set<int> >::iterator iter_ConvexChains = laserSensing.ConvexChains.begin() + control._indexClusters_reactive;
							// cv::Point2f pt, ptObsNear;
							cv::Point2f pt;
							if ( (*iter_ConvexChains).size() > 1)
							{
								std::vector<cv::Point2f> pts;
								pts.reserve( (*iter_ConvexChains).size() );
								std::set<int>::iterator iter_polygonalchain;
								for( iter_polygonalchain = iter_ConvexChains->begin(); iter_polygonalchain != iter_ConvexChains->end(); iter_polygonalchain++ )
								{
									int i = *iter_polygonalchain;
									cv::Point2f pt;
									navi.scan2CartesianInMeter(las, i, pt);
	//								navi.scan2Cartesian(las, i, pt);
	//								pt.x = pt.x/1000;	// change to metre
	//								pt.y = pt.y/1000;
									pts.push_back(pt);
								}

								// BSplineCubic bspline(pts, 0.05);	// the path is in meter
								// navi.drawBSpline(width_area, height_area, bspline.curve_BSpline, I_laserReactiveObstacle);
								laserSensing.bspline.calculateCubicCurve(pts, 0.05);

								// search for the nearest point on the path
								int i_near = 0;     // for the nearest point
								double dis_nearcurve = 5;	// in meter
								double dis = 0;
								for (int i = 0; i < laserSensing.bspline.curve_BSpline.size(); i++)
								{
									pt.x = laserSensing.bspline.curve_BSpline[i].p.x;
									pt.y = laserSensing.bspline.curve_BSpline[i].p.y;

									dis = distance2d(pt, cv::Point2f(0, 0));
									if (dis < dis_nearcurve)
									{
										dis_nearcurve = dis;
										dis_robotTopath = dis_nearcurve;
										i_near = i;
									}
								}
								// navi.drawBSpline(width_area, height_area, laserSensing.bspline.curve_BSpline, I_laserReactiveObstacle); // do not draw Bspline
								pt.x = laserSensing.bspline.curve_BSpline[i_near].p.x *1000;
								pt.y = laserSensing.bspline.curve_BSpline[i_near].p.y *1000;
								control._nearPointOnBspline.x = pt.x;
								control._nearPointOnBspline.y = pt.y;
								//navi.drawNearestPoint( las, width_area, height_area, pt, I_laserReactiveObstacle);		// do not draw nearest points
								// cv::imshow(" laser reactive obstacle ", I_laserReactiveObstacle);

								// transform the robot coordinate to the nearest curve point coordinate
								// set the nearest point as the relative coordinate
								Pose p_relative;
								p_relative.p.x = laserSensing.bspline.curve_BSpline[i_near].p.x;
								p_relative.p.y = laserSensing.bspline.curve_BSpline[i_near].p.y;
								p_relative.phi = laserSensing.bspline.curve_BSpline[i_near].phi;
								Transform2D trans2d(p_relative);		     // p_relative as the base
								// transform the robot coordinate (world frame) pose to the relative coordinate
								Pose p_robinrelative;
								p_robinrelative.p.x = 0;
								p_robinrelative.p.y = 0;
								p_robinrelative.phi = 0;
								trans2d.transform_to_relative(p_robinrelative);

	//							// control law
								double delta_2 = 0, alpha_2, chi_2, k_2;
								k_2 = 0.9;
								pt.x = laserSensing.bspline.curve_BSpline[i_near].p.x;
								pt.y = laserSensing.bspline.curve_BSpline[i_near].p.y;
								double delta_abs = std::abs(distance2d(pt, cv::Point2f(0, 0)) - robot._disFollowing);
								if( p_robinrelative.p.y <= 0 && distance2d(pt, cv::Point2f(0, 0)) >= robot._disFollowing )
									delta_2  = delta_abs;
								if ( p_robinrelative.p.y <= 0 && distance2d(pt, cv::Point2f(0, 0)) < robot._disFollowing )
									delta_2 = -delta_abs;
								if ( p_robinrelative.p.y > 0 && distance2d(pt, cv::Point2f(0, 0)) >= robot._disFollowing )
									delta_2 = -delta_abs;
								if ( p_robinrelative.p.y > 0 && distance2d(pt, cv::Point2f(0, 0)) < robot._disFollowing )
									delta_2 = delta_abs;

								// angle calculation
								alpha_2 = laserSensing.bspline.curve_BSpline[i_near].phi;
								alpha_2 = pi_to_pi(alpha_2);
								chi_2 = 0;
								Va_2 = -Vl_1 * (k_2 * delta_2 + alpha_2 + 2 * k_2 * std::sin(alpha_2) - chi_2 * std::cos(alpha_2));

							}	// end if ( (*iter_ConvexChains).size() > 1)	// fitting the boundary
						}		// // do reactive
						else
						{
							// robot._motionState.motionStateCurrent = MotionState::TARGET_FINDING;
							robot._motionState.motionStateCurrent    = MotionState::STOP;	// stop the speed
							robot._motionState.motionStateRemembered = MotionState::TARGET_FINDING;		// after stop do target finding
							control._b_leave = false;
							control._b_collision = false;
						}

						/**
						 *  2.3.3 leave condition
						 */
						double dis_escape;
						if ( control._b_collision == true )
						{
							// for counterclockwise case
							if ( Va_2 < Va_1  )
							{
								control._b_leave = true;
								dis_escape = dis_robotTopath;
							}
							else
								control._b_leave = false;

						}

//						if ( control._b_collision == true )
//						{
//							if ( control._b_clockwise == false )	// for counterclockwise case
//							{
//								if ( Va_2 < Va_1  )
//								{
//									control._b_leave = true;
//									dis_escape = dis_robotTopath;
//								}
//								else
//									control._b_leave = false;
//							}
//							else									// for counterclockwise case
//							{
//								if ( Va_2 > Va_1  )
//								{
//									control._b_leave = true;
//									dis_escape = dis_robotTopath;
//								}
//								else
//									control._b_leave = false;
//							}
//						}

//						if ( control._b_leave == false )
//						{
//							if ( control._b_clockwise == false )	// for counterclockwise case
//							{
//								if ( Va_2 < Va_1  )
//								{
//									control._b_leave = true;
//									dis_escape = dis_robotTopath;
//								}
//							}
//							else									// for counterclockwise case
//							{
//								if ( Va_2 > Va_1  )
//								{
//									control._b_leave = true;
//									dis_escape = dis_robotTopath;
//								}
//							}
//						}

						/*
						 * 	2.3.4 global controller
						 */
						double mu = 0;

						// if in the switch case, check the leave condition
						if ( dis_robotTopath < robot._disReactive + robot._radius
								&& Va_2 < Va_1 )										// no escape for counterclockwise case
						{
							control._b_leave = true;
							dis_escape = dis_robotTopath;
							//b_collision = false;
						}

						// mu function
						if ( control._b_collision == false )
							mu = 0;
						if ( control._b_collision == true &&  dis_robotTopath > robot._disFollowing
								&& dis_robotTopath < robot._disReactive && control._b_leave == false )
							mu = ( dis_robotTopath -robot._disReactive )/(robot._disFollowing - robot._disReactive);
						if ( dis_robotTopath > robot._disStop && control._b_leave == true )
							mu = 0;
						if ( dis_robotTopath < robot._disFollowing && control._b_leave == false )
							mu = 1;

//						if ( control._b_collision == false )
//							mu = 0;
//						if ( control._b_collision == true &&  dis_robotTopath > robot._disFollowing
//								&& dis_robotTopath < robot._disReactive && control._b_leave == false )
//							mu = ( dis_robotTopath -robot._disReactive )/(robot._disFollowing - robot._disReactive);
//						else
//							mu = 1;

						// corperate the two controllers
						expectedVa = (1-mu) * Va_1 + mu * Va_2;

					}		// if ( motionState == MotionState::OBSTACLE_AVOIDANCE )

#endif

					loopNumber++;		// computer the loop
#ifndef Simulation
				}	// if all the data are ok
				delete img;
				delete img2;
			}	// if all the data are not NULL
#endif
			ss.str("");
			ss << std::setfill('0') << std::setw(6) << loopNumber;
			t2 = clock();
//			std::cout << " processing stereo images " << ss.str().c_str() << " in " << (float)( t2 - t1 )<< " ms " << std::endl;
//			std::cout << " processing stereo images " << ss.str().c_str() << " in " << (float)( t2 - t1 )*1000  /  (float)CLOCKS_PER_SEC<< " ms " << std::endl;
			// show results, check before using
//			ss.str("");
//			ss<< "b_start: " << b_start;
//			cv::putText( M_Coordinate, ss.str(), cv::Point(50,100), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,255,0), 1, 8, false);

			/**
			 * draw the results
			 */
//			void MyDraw::drawTargetDirection( const float theta,	// direction
//											  const float r,		// range of the direction line
//											  const float scale_w,
//											  const float scale_h,
//											  cv::Mat &I)
//
//			myDraw.drawTargetDirection()

			// draw laser results
			I_laser = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
			myDraw.drawLaserCoordinate(width_area, height_area, I_laser);
			myDraw.drawLaser(las, width_area, height_area, I_laser);							// raw data
			I_laserNoiseFiltering = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
			myDraw.drawLaserCoordinate(width_area, height_area, I_laserNoiseFiltering);
			myDraw.drawLaser(las, width_area, height_area, laserSensing.status, I_laserNoiseFiltering);	// noise filtered
//			I_laserClustering = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//			myDraw.drawLaserCoordinate(width_area, height_area, I_laserClustering);
//			myDraw.drawLaser(las, width_area, height_area, laserSensing.Clusters, I_laserClustering);		// clustering
//			I_laserMerging = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
//			myDraw.drawLaserCoordinate(width_area, height_area, I_laserMerging);
//			myDraw.drawLaser(las, width_area, height_area, laserSensing.Clusters, I_laserMerging);		// merging

			I_laserSegments = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
			myDraw.drawLaserCoordinate(width_area, height_area, I_laserSegments);

			I_laserClusteringAndMerging = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
			myDraw.drawLaserCoordinate(width_area, height_area, I_laserClusteringAndMerging);
			// myDraw.drawLaser(las, width_area, height_area, laserSensing.Clusters, I_laserClusteringAndMerging);		// clustering and merging

			I_laserPolygonalChains = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
			//I_laserClustering.copyTo(I_laserPolygonalChains);
			//myDraw.drawLaserCoordinate(width_area, height_area, I_laserPolygonalChains);
			//myDraw.drawLaser(las, width_area, height_area, laserSensing.Clusters, laserSensing.ConvexChains, I_laserPolygonalChains);	// Polygonal Chains

			// myDraw.drawLaserCoordinate(width_area, height_area, I_laserPolygonalChains);
			//myDraw.drawPolygonalChains( las, width_area, height_area, laserSensing.Clusters, laserSensing.PolygonalChains, I_laserPolygonalChains);
			I_laserConvexChains = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
			// myDraw.drawLaserCoordinate(width_area, height_area, I_laserConvexChains);
			//myDraw.drawConvexChains( las, width_area, height_area, laserSensing.Clusters, laserSensing.ConvexChains, I_laserConvexChains);

			myDraw.drawLaserResults( las, width_area, height_area, laserSensing._Segments, laserSensing.Clusters, laserSensing.PolygonalChains,
										laserSensing.ConvexChains, I_laserSegments, I_laserClusteringAndMerging,
										I_laserPolygonalChains, I_laserConvexChains);

			// draw obstacle avoidance (predicted trajectory)
			I_laserReactiveObstacle = cv::Mat(cv::Size(width_winLaser,height_winLaser), CV_8UC3, cv::Scalar::all(255));
			myDraw.drawLaserCoordinate(width_area, height_area, I_laserReactiveObstacle);
			myDraw.drawPredictedTrajecotry(control._trajectoryPrediction,myDraw._wScale_meter2laserwindow,myDraw._hScale_meter2laserwindow, (float)robot._radius, control._b_collision, I_laserReactiveObstacle);
			myDraw.drawObstacleReactive(las, width_area, height_area, laserSensing.Clusters, laserSensing.ConvexChains, control._b_collision, control._indexClusters_reactive, I_laserReactiveObstacle);
			//navi.drawNearestPoint( las, width_area, height_area, control._nearPointOnBspline, I_laserReactiveObstacle);
			if ( control._b_collision == true )
			{
				myDraw.drawBSpline(width_area, height_area, laserSensing.bspline.curve_BSpline, I_laserReactiveObstacle);
				myDraw.drawNearestPoint( las, width_area, height_area, control._nearPointOnBspline, I_laserReactiveObstacle );
			}

			// draw the estimatd features
			myDraw.drawFeatureEstimation( objectDetection.scene_corners_current, objectDetection.pt_gravityCenter_current, M1_color);

			/**
			 *  show the results
			 */
			// image of the left camera
			s.str("");
			s << "I1_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//cv::putText( M1_color, "image of the left camera", cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,255,0), 1, 8, false);
			if ( !M1_color.empty() ) cv::imshow( "img", M1_color);	cv::imwrite( img1_path, M1_color);
			// visual tracking
			s.str("");
			s << "track_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//cv::putText( I_track, "track in the left image", cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,255,0), 1, 8, false);
			if ( !I_track.empty() ) cv::imshow( "I_tracking",  I_track);	cv::imwrite( img1_path, I_track);
			// draw the estimatd features
			//navi.drawFeatureEstimation(objectDetection.scene_corners_current, robot._motionState.motionStateCurrent, M1_color, I_track, I_featureEstimation);
			//cv::imshow( "Visual features estimation",  I_featureEstimation);
			//s.str("");
			//s << "featureEstimated_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			//img1_path = img_folder + s.str();
			//cv::imwrite( img1_path, I_featureEstimation);
			// image of the right camera
			s.str("");
			s << "I2_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//cv::putText( M2_color, "image of the right camera", cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,255,0), 1, 8, false);
			if ( !M2_color.empty() ) cv::imshow( "img2", M2_color);	 // cv::imwrite( img1_path, M2_color);
			// odometry
			s.str("");
			s << "odometry_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			cv::putText( M_Coordinate, MotionState::get_State(robot._motionState.motionStateCurrent), cv::Point(50,100), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar::all(0), 1, 8, false);
			//cv::putText( M_Coordinate, "stereo visual odometry", cv::Point(50,400), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,255,0), 1, 8, false);
			if ( !M_Coordinate.empty() ) cv::imshow( "M_Coordinate", M_Coordinate );	cv::imwrite( img1_path, M_Coordinate);
			// origin data from laser
			s.str("");
			s << "laser_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			if ( !I_laser.empty() )	cv::imshow("laser", I_laser);
			cv::imwrite( img1_path, I_laser);
			// noise filtering
			s.str("");
			s << "laserNoiseFiltering_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//if ( !I_laserNoiseFiltering.empty() )	cv::imshow("laser", I_laser);
			cv::imwrite( img1_path, I_laserNoiseFiltering);
			// segmentations
			s.str("");
			s << "laserSegmentations_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//if ( !I_laserMerging.empty() )	cv::imshow("laser", I_laserClusteringAndMerging);
			cv::imwrite( img1_path, I_laserSegments);
			// Clustering and merging
			s.str("");
			s << "laserClusteringAndMerging_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//if ( !I_laserMerging.empty() )	cv::imshow("laser", I_laserClusteringAndMerging);
			cv::imwrite( img1_path, I_laserClusteringAndMerging);
			// Polygonal Chains
			s.str("");
			s << "laserPolygonalChains_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//cv::putText( I_laser, "Polygonal chains", cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,255,0), 1, 8, false);
			//if ( !I_laserPolygonalChains.empty() ) 	cv::imshow("laser", I_laserPolygonalChains);
			cv::imwrite( img1_path, I_laserPolygonalChains);
			// Convex Chains
			s.str("");
			s << "laserConvexChains_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//cv::putText( I_laser, "Convex Chains", cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255,255,255,0), 1, 8, false);
			// if ( !I_laserConvexChains.empty() ) 	cv::imshow("laser", I_laser);
			cv::imwrite( img1_path, I_laserConvexChains);
			// reactive obstacle
			s.str("");
			s << "obstacle_" << std::setw(6) << std::setfill('0') << loopNumber << ".png";
			img1_path = img_folder + s.str();
			//cv::putText( I_laserReactiveObstacle, "obstacle for reactive", cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar::all(0), 1, 8, false);
			if ( !I_laserReactiveObstacle.empty() ) cv::imshow("laser reactive obstacle", I_laserReactiveObstacle);	cv::imwrite( img1_path, I_laserReactiveObstacle);

			/**
			 * 	send the velocity
			 */
#ifndef Simulation
			//	Low pass filter to linear velocity
			double k_LPF_vl = 0.1;
			if ( robot._motionState.motionStateCurrent == MotionState::STOP  || robot._motionState.motionStateCurrent == MotionState::END )
			{
				expectedVl = 0;
				k_LPF_vl = 0.5;
			}
			motionControl::lowPassFilter(k_LPF_vl, expectedVl, _newVl);

			//	Low pass filter to angular velocity
			double k_LPF_va;
			if ( robot._motionState.motionStateCurrent == MotionState::TARGET_TRACKING )
				 k_LPF_va = 0.01;	// 0.02
			else
				k_LPF_va = 0.1;
			if ( robot._motionState.motionStateCurrent == MotionState::STOP || robot._motionState.motionStateCurrent == MotionState::END )
			{
				expectedVa = 0;
				k_LPF_va = 0.5;
			}
			motionControl::lowPassFilter(k_LPF_va, expectedVa, _newVa);

			// send the velocity
			//std::cout << "_newVl: " << _newVl <<  "		_newVa: " << _newVa*180/PI << std::endl;
		    rl->setVlVa(_newVl, _newVa, false);

#endif
		    // check the stop command
			if( cv::waitKey(20) == 1048689 )
			{
#ifndef Simulation
				//rl->setVlVa(0.0, 0.0, false);
				time_0 = clock();
				robot._motionState.motionStateCurrent = MotionState::END;
#endif
			}
			// stop the robot
			if( robot._motionState.motionStateCurrent == MotionState::STOP && _newVl < 0.005 && _newVa < PI/100 )
			{

				_newVl = 0;
				_newVa = 0;
				rl->setVlVa(0.0, 0.0, false);
			}

			/**
			 *  generate the log
			 */
#ifdef	Generate_Log
			lRobotPose << Odometries[loopNumber].p.x << "\t" << Odometries[loopNumber].p.y << "\t"
					<< Odometries[loopNumber].phi  << std::endl;
			lControlInput << _newVl << "\t" << _newVa*180/PI << std::endl;
			lRobotState << MotionState::get_State(robot._motionState.motionStateCurrent) << std::endl;
			lCameraInWorld << w_T_c[0][0] << "\t" <<  w_T_c[0][1] << "\t" <<  w_T_c[0][2] << "\t" <<  w_T_c[0][3] << "\t"
								<<  w_T_c[1][0] << "\t" <<  w_T_c[1][1] << "\t" <<  w_T_c[1][2] << "\t" <<  w_T_c[1][3] << "\t"
								<<  w_T_c[2][0] << "\t" <<  w_T_c[2][1] << "\t" <<  w_T_c[2][2] << "\t" <<  w_T_c[2][3] << "\t"
								<<  w_T_c[3][0] << "\t" <<  w_T_c[3][1] << "\t" <<  w_T_c[3][2] << "\t" <<  w_T_c[3][3] <<  std::endl;

			lLaserData << 0.006132813 << "\t" << las->size << "\t"  << 1000;
			for (int i=0; i < las->size; i++ )
				lLaserData << "\t"  << las->data[i];
			lLaserData << std::endl;

			for (int i=0; i < laserSensing.status.size(); i++ )
				lLaserFiltered << laserSensing.status[i] << "\t";
			lLaserFiltered << std::endl;

			for( int i = 0; i < objectDetection.scene_corners_current.size(); i++ )
			{
				lTargetFeatures << objectDetection.scene_corners_current[i].x << "\t" << objectDetection.scene_corners_current[i].y << "\t";
			}
			lTargetFeatures << std::endl;

			// B-spline data points
			std::vector<cv::Point2f>::iterator iter_BSplineDataPoints;
			lBsplinesDatapts << laserSensing.bspline.pts_sample.size() << "\t";
			for ( iter_BSplineDataPoints = laserSensing.bspline.pts_sample.begin(); iter_BSplineDataPoints != laserSensing.bspline.pts_sample.end(); iter_BSplineDataPoints++ )
			{
				cv::Point2f pt1;	// in cartesian coordiante in mm
				pt1.x = (*iter_BSplineDataPoints).x;
				pt1.y = (*iter_BSplineDataPoints).y;
				lBsplinesDatapts << pt1.x << "\t" << pt1.y << "\t";
			}
			lBsplinesDatapts << std::endl;

			// B-spline
			std::vector<Pose>::iterator iter_BSpline;
			lBspline<< laserSensing.bspline.curve_BSpline.size() << "\t";
			for ( iter_BSpline = laserSensing.bspline.curve_BSpline.begin(); iter_BSpline != laserSensing.bspline.curve_BSpline.end(); iter_BSpline++ )
			{
				cv::Point2f pt1;	// in cartesian coordiante in mm
				pt1.x = (*iter_BSpline).p.x;
				pt1.y = (*iter_BSpline).p.y;
				lBspline << pt1.x << "\t" << pt1.y << "\t";
			}
			lBspline << std::endl;

#endif		// #ifdef	Generate_Log

			/**
			 *  print out
			 */
			std::cout << " current state : " << MotionState::get_State(robot._motionState.motionStateCurrent) << "	"
					<< MotionState::get_State(robot._motionState.motionStateRemembered) << std::endl;

			// end the project
			if( robot._motionState.motionStateCurrent == MotionState::END && _newVl < 0.005 && _newVa < PI/100 )
			{

				_newVl = 0;
				_newVa = 0;
				rl->setVlVa(0.0, 0.0, false);

				// check before use release
				cv::destroyWindow( "M1" );
				cv::destroyWindow( "M2" );
				cv::destroyWindow( "M_Coordinate" );

				// release tracking
				if (esmTrack.images != NULL)
				{
					FreeTrack(&esmTrack);
					esmTrack.images = NULL;
					esmTrack.trackdata = NULL;
				}

				return 0;
			}

			// loopNumber++;		// computer the loop

		}
		catch( ... ) {
			std::cout << "Try catch " << std::endl;
		}
	}

	return 0;
}




