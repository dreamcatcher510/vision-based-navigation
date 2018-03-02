#include <iostream>
#include <algorithm>	// mathematics
#include <time.h>

// for PathFileExists
//#pragma comment(lib, "shlwapi.lib")
//#include <shlwapi.h>

//#include "cv.h"
//#include "highgui.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "demo_odo.h"
//#include "matcher.h"
#include "visualodometry.h"
#include "matcherMex.h"
#include "visualOdometryMex.h"
#include "Navigation.h"

#define USEC_PER_SEC 1000000L

//static Matcher *M;
//static VisualOdometry *VO;

int DemoOdo::init( std::string m, std::string f )
{
	if ( m != "flow" &&  m != "stereoMatch" &&  m != "mono" && m != "stereo" )
	{
		std::cout << "incorrect value for odometry model ... " << std::endl;
		return -1;
	}
	else
	{
		method = m;
	}

	if ( f.empty() )
	{
		std::cout << "files folder is empty ... " << std::endl;
		return -1;
	}
	else
	{
		files_folder = f;
	}

	// define the color
	colors[0] =  (0,0,255);		// 0: red
	colors[1] =  (0,128,255);
	colors[2] =  (0,255,255);
	colors[3] =  (0,255,0);		// 3: green
	colors[4] =  (255,128,0);
	colors[5] =  (255,255,0);
	colors[6] =  (255,0,0);		// 6: blue
	colors[7] =  (255,0,255);
	colors[8] =  (255,255,255);
	colors[9] =  (0,255,255, 0); // yellow

	return 0;
}
int DemoOdo::initCalib(double f, double cu, double cv, double base)
{
	calib_f = f;
	calib_cu = cu;
	calib_cv = cv;
	calib_b = base;
	return 0;
}

int DemoOdo::runStereo_rectifiedInput()
{
	///*****************************************************************************/
	///* parameters
	///*****************************************************************************/
	////// before 2012_09_19
	////// intrinsic parameters
	////cv::Mat cameraMatrix_1 = (cv::Mat_<double>(3,3) << 594.08601, 		0, 304.97190,
	////														  0, 594.59180, 230.04212,
	////														  0,	     0,			1);
	////cv::Mat distCoeffs_1 = (cv::Mat_<double>(1,5) << -0.47396, 0.23791, 0.00188, 0.00118, 0.00000);
	////cv::Mat cameraMatrix_2 = (cv::Mat_<double>(3,3) << 599.69210, 		0, 336.97846,
	////														  0, 599.80256, 216.90643,
	////														  0,	     0,			1);
	////cv::Mat distCoeffs_2 = (cv::Mat_<double>(1,5) << -0.47428, 0.22638, 0.00193, -0.00356, 0.00000);

	////// extrinsic parameters
	////cv::Mat r = (cv::Mat_<double>(1,3) << 0.00953, 0.02114, -0.01460);
	////cv::Mat R;
	////cv::Rodrigues(r, R);
	////cv::Mat T = (cv::Mat_<double>(3,1) << -0.08347457, -0.00045096, 0.00170208);		// in metric

	//// after 2012_09_19
	//// intrinsic parameters
	//cv::Mat cameraMatrix_1 = (cv::Mat_<double>(3,3) << 593.19704, 		0, 292.73890,
	//														  0, 593.07076, 224.55454,
	//														  0,	     0,			1);
	//cv::Mat distCoeffs_1 = (cv::Mat_<double>(1,5) << -0.44931, 0.17271, 0.00159, 0.00411, 0.00000);
	//cv::Mat cameraMatrix_2 = (cv::Mat_<double>(3,3) << 596.47038, 		0, 304.79233,
	//														  0, 597.92401, 213.82504,
	//														  0,	     0,			1);
	//cv::Mat distCoeffs_2 = (cv::Mat_<double>(1,5) << -0.50426, 0.33753, 0.00127, 0.00229, 0.00000);

	//// extrinsic parameters
	//cv::Mat r = (cv::Mat_<double>(1,3) << -0.00688, 0.02052, -0.02026);
	//cv::Mat R;
	//cv::Rodrigues(r, R);
	//cv::Mat T = (cv::Mat_<double>(3,1) << -0.08002273, 0.00042560, 0.00214352);		// in metric

	//
	//// stereo rectification
	//// rectify images
	//cv::Mat R1, R2, P1,P2, Q;
	//cv::Rect validPixROI1, validPixROI2;
	//cv::Size imageSize(640,480);
	//cv::Mat rmap[2][2];
	//stereoRectification(cameraMatrix_1, distCoeffs_1, cameraMatrix_2, distCoeffs_2,
	//					imageSize, R, T, R1, R2, P1, P2, Q, imageSize, validPixROI1,  validPixROI2, rmap);

	////// odometry objects
	//double f     = P1.at<double>(0,0);
 //   double cu    = P1.at<double>(0,2);
 //   double cv    = P1.at<double>(1,2);
 //   double base  = -P2.at<double>(0,3)/P2.at<double>(0,0);			// here is for stereo
 //   double pitch = 0;

	////// odometry objects		// for 2010_03_09_drive_0019
	////double f     = 645.2401;
 ////   double cu    = 635.9587;
 ////   double cv    = 194.1291;
 ////   double base  = 0.5707;			// for stereo
 ////   double pitch = 0;				// for mono

	//initCalib(f, cu, cv, base);

	//// matching parameters
 //   int nms_n = 3;					// non-max-suppression: min. distance between maxima (in pixels)
	//int nms_tau = 50;				// non-max-suppression: interest point peakiness threshold
	//int match_binsize = 50;			// matching bin width/height (affects efficiency only)
	//int match_radius = 200;			// matching radius (du/dv in pixels)
	//int match_disp_tolerance = 1;   // du tolerance for stereo matches (in pixels)
	//int outlier_disp_tolerance = 5; // outlier removal: disparity tolerance (in pixels)
	//int outlier_flow_tolerance = 5; // outlier removal: flow tolerance (in pixels)
	//int multi_stage = 1;			// 0=disabled,1=multistage matching (denser and faster)
	//int half_resolution = 1;		// 0=disabled,1=match at half resolution, refine at full resolution
	//int refinement = 1;				// refinement (0=none,1=pixel,2=subpixel)

	//// bucketing parameters
	//int max_features = 10;
	//float bucket_width = 20;
	//float bucket_height = 20;
	//float deltaT = 0.1f;

	////// bucketing parameters
	////int max_features = 1;
	////float bucket_width = 240;
	////float bucket_height = 100;
	////float deltaT = 0.1f;

	//// init matcher
	//MatcherMex matcherMex;
	//matcherMex.matcherMex( "init", nms_n,nms_tau,match_binsize,match_radius,match_disp_tolerance,
	//						outlier_disp_tolerance,outlier_flow_tolerance,multi_stage,half_resolution,refinement );

	//// init odometry objects
	//VO = new VisualOdometry();
 //   VO->setCalibration(f,cu,cv,base,pitch);
	//VisualOdometryMex visualOdometryMex;

	//// init transformation matrix array
	//std::vector<Matrix> Tr_total;
	//Matrix matrix(4,4);
	//matrix.eye();
	//Tr_total.push_back (matrix);

	//// create images
	//cv::Mat M1_origi, M2_origi;
	//cv::Mat M1, M2;
	//cv::Mat M1_color, M2_color;
	//cv::Mat M_Coordinate(cv::Size(500,500),CV_8UC3);
	//float range_x = 14;
	//float range_z = 14;
	//double ratioW_draw = M_Coordinate.cols/range_x;	// width for x meters
	//double ratioH_draw = M_Coordinate.rows/range_z;	// heighth for y meters
	//cv::Point point_center( M_Coordinate.cols/2, M_Coordinate.rows/2 );	 // image coordinate
	//plotCoordinate( M_Coordinate, point_center, ratioW_draw, ratioH_draw, range_x, range_z );

	////// create images
	////cv::Mat M1, M2, M1p, M2p, M_stackedHori, M_stackedVert, M_stackedQuad;
	////cv::Mat M1_origi, M2_origi;
	////cv::Mat M1_color, M2_color, M1p_color, M2p_color, M_stackedHori_color,  M_stackedVert_color, M_stackedQuad_color,  M_stackedQuad_color_half;
	////cv::Mat M_Coordinate(cv::Size(500,500),CV_8UC3);
	////double ratioW_draw = M_Coordinate.cols/15;	// width for x meters
	////double ratioH_draw = M_Coordinate.rows/15;	// heighth for y meters
	////cv::Point point_center( M_Coordinate.cols/2, M_Coordinate.rows/2 );	 // image coordinate
	////plotCoordinate( M_Coordinate, point_center, ratioW_draw, ratioH_draw );

	//// create windows
	//cv::namedWindow( "M1", CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "M_Coordinate", CV_WINDOW_AUTOSIZE );

	//// for reading image
	//std::stringstream ss;
	//std::string img_path;

	//// for transformation
	//int failure = 0;		// failure = n : count the times failed, failure == 0 not failed
	//bool success = NULL;
	//int k = 1;	// the index of Tr_total (transformation matrix)
	//unsigned int num_frame = 0;

	//while (true)
	//{
	//	// read current stereo images
	//	ss.str("");
	//	ss << std::setfill('0') << std::setw(6) << num_frame;
	//	img_path = files_folder + "I1_" + ss.str() + ".png";
	//	if( PathFileExists(img_path.c_str()) )
	//	{
	//		M1_origi = cv::imread( img_path, 0 );
	//		cv::remap(M1_origi, M1, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
	//		cv::cvtColor( M1, M1_color, CV_GRAY2BGR);
	//	}
	//	else
	//	{
	//		std::cout << "Error reading current left image ..." << std::endl;
	//		break;
	//	}
	//	ss.str("");
	//	ss << std::setfill('0') << std::setw(6) << num_frame;
	//	img_path = files_folder + "I2_" + ss.str() + ".png";
	//	if( PathFileExists(img_path.c_str()) )
	//	{
	//		M2_origi = cv::imread( img_path, 0 );
	//		cv::remap(M2_origi, M2, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
	//		cv::cvtColor( M2, M2_color, CV_GRAY2BGR);
	//	}
	//	else
	//	{
	//		std::cout << "Error reading current right image ..." << std::endl;
	//		break;
	//	}

	//	clock_t t1,t2;
	//	t1 = clock();
	//	/*****************************************************************************/
	//	/* proccessing
	//	/*****************************************************************************/
	//	// transform opencv image structure to libviso2 image structure
	//	// get pointers to input data
	//	uint8_t* I1          = (uint8_t*)M1.ptr(0);	// returns pointer to 0-th row
	//	int32_t I1_size[2] = { M1.cols, M1.rows };
	//	const int32_t *dims1 = I1_size;
	//	uint8_t* I2          = (uint8_t*)M2.ptr(0);	// returns pointer to 0-th row
	//	int32_t I2_size[2] = { M2.cols, M2.rows };
	//	const int32_t *dims2 = I2_size;

	//	// keep previous stereo images, if visual odometry estimate failed
	//	if ( failure == 0 )		// if not failed
	//		matcherMex.matcherMex( "push", I1, dims1, I2, dims2 );
	//	else
	//		matcherMex.matcherMex( "replace", I1, dims1, I2, dims2 );

	//	// start matching after reading 2nd stereo frames
	//	if ( num_frame > 1 )
	//	{
	//		// match features and also get a bucketed version
	//		// matching method: 0 = flow, 1 = stereo, 2 = quad matching
	//		matcherMex.matcherMex( "match", 2);
	//		std::vector<Matcher::p_match> matches_full;	// full matches
	//		matcherMex.matcherMex( matches_full, "getmatches", 2 );
	//		std::vector<Matcher::p_match> matches_bucketing;	// bucketing matches
	//		matcherMex.matcherMex( "bucketing", max_features,bucket_width,bucket_height);
	//		matcherMex.matcherMex( matches_bucketing, "getmatches", 2 );

	//		// estimate egomotion, failure+1 is the number of frames between matches
	//		success = visualOdometryMex.visualOdometryMex( VO, "update", (failure+1)*deltaT, matches_bucketing, "stereo" );

	//		// get inliers
	//		std::vector<int32_t> inliers = VO->getInliers();

	//		// if visual odometry estimate was successful
	//		Matrix Tr(4,4);
	//		if ( success == true )
	//		{
	//			 // get transformation matrix (state)
	//			 // get state
	//			 //Tr = ~VO->getTransformation();	 // VO->getTransformation() return the transformation matrix of the previous frame
	//				//							 // respect to the current frame
	//				//							 // ~ to get the transform matrix of Tr also the previous frame respect to the current frame
	//			Tr = VO->getTransformation();		// it is better not use transform ~, it already output normal format
	//			 failure = 0;
	//		}
	//		// otherwise keep current position
	//		// (this happens if motion is very small)
	//		else
	//		{
	//			failure = failure+1;
	//		}

	//		//std::cout << "Tr :"  << std::endl;
	//		//for(int i = 0; i < 4; i++)
	//		//{
	//		//	for(int j = 0; j < 4; j++)
	//		//	{
	//		//		std::cout << Tr.val[i][j] << "	" ;
	//		//	}
	//		//	std::cout << std::endl;
	//		//}

	//		Tr.inv();								// transformation matrix of the current frame respect to the previous frame
	//		// accumulate total transformation
	//		Tr_total.push_back( Tr_total[k-1]*Tr );	// Tr_total[k-1]*Tr get the transformation matrix of the current frame respect to the first frame
	//
	//
	//		std::cout << "Tr_total(current) :"  << std::endl;
	//		for(int i = 0; i < 4; i++)
	//		{
	//			for(int j = 0; j < 4; j++)
	//			{
	//				std::cout << Tr_total[Tr_total.size()-1].val[i][j] << "	  ";
	//			}
	//			std::cout << std::endl;
	//		}
	//
	//		k=k+1;

	//		// draw the results
	//		plotMatch( M1_color, matches_bucketing, 0);
	//		plotTrajectory( M_Coordinate, Tr_total[Tr_total.size()-1],point_center, ratioW_draw, ratioH_draw, range_x, range_z );
	//	}


	//	/*****************************************************************************/
	//	/* proccessing finish
	//	/*****************************************************************************/
	//	t2 = clock();
	//	std::cout << " processing stereo images " << ss.str().c_str() << " in " << (float)( t2 - t1 )*1000  /  (float)CLOCKS_PER_SEC<< " ms " << std::endl;

	//	// show results, check before using
	//	if ( !M1_color.empty() ) cv::imshow( "M1", M1_color);
	//	if ( !M2_color.empty() ) cv::imshow( "M2", M2_color);
	//	if ( !M_Coordinate.empty() ) cv::imshow( "M_Coordinate", M_Coordinate );
	//	cv::waitKey(10);

	//	// check before use release
	//	if ( !M1.empty() )	M1.release();
	//	if ( !M1_color.empty() )	M1_color.release();
	//	if ( !M2.empty() )	M2.release();
	//	if ( !M2_color.empty() )	M2_color.release();

	//	num_frame++ ;
	//}


	//// check before use release
	//cv::destroyWindow( "M1" );
	//cv::destroyWindow( "M2" );
	//if ( !M1.empty() )	M1.release();
	//if ( !M1_color.empty() )	M1_color.release();
	//if ( !M2.empty() )	M2.release();
	//if ( !M2_color.empty() )	M2_color.release();
	//cv::destroyWindow( "M_Coordinate" );
	//if ( !M_Coordinate.empty() )	M_Coordinate.release();

	////close matcher
	//matcherMex.matcherMex( "close" );
	//visualOdometryMex.visualOdometryMex( VO, "close" );

	return 0;
}

// stereo rectification
int DemoOdo::stereoRectification(const cv::Mat cameraMatrix1, const cv::Mat distCoeffs1, const cv::Mat cameraMatrix2, const cv::Mat distCoeffs2,
						const cv::Size imageSize, const cv::Mat R, const cv::Mat T, cv::Mat &R1, cv::Mat &R2, cv::Mat &P1, cv::Mat &P2, cv::Mat &Q,
						const cv::Size newImageSize, cv::Rect &validPixROI1, cv::Rect &validPixROI2, cv::Mat rmap[][2])
{
	//cv::stereoRectify( cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q,
	//	cv::CALIB_ZERO_DISPARITY, 1, newImageSize, &validPixROI1, &validPixROI2 );

		cv::stereoRectify( cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, newImageSize, &validPixROI1, &validPixROI2 );

	// save rectified parameters
	cv::FileStorage fs("extrinsics.yml", CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		std::cout << "Error: can not save the intrinsic parameters\n";

	cv::Mat om, om1, om2;
	cv::Rodrigues(R, om);
	cv::Rodrigues(R1, om1);
	cv::Rodrigues(R2, om2);

	//// print out parameters
	//std::cout  << "R\n" << R << std::endl;
	//std::cout  << "T\n" << T << std::endl;
	//std::cout  << "R1\n" << R1 << std::endl;
	//std::cout  << "R2\n" << R2 << std::endl;
	//std::cout  << "P1\n" << P1 << std::endl;
	//std::cout  << "P2\n" << P2 << std::endl;
	//std::cout  << "Q\n" << Q << std::endl;
	//std::cout  << "om\n" << om << std::endl;
	//std::cout  << "om1\n" << om1 << std::endl;
	//std::cout  << "om2\n" << om2 << std::endl;

	//Precompute maps for cv::remap()
	cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	//std::cout << " stereo rectificaiton finished ..." << std::endl;
	return 0;
}

// stack two images horizontally
cv::Mat DemoOdo::stack_imgsHori( cv::Mat I1, cv::Mat I2 )
{
	cv::Mat stacked = cv::Mat::zeros( cv::Size( I1.cols + I2.cols, std::max(I1.rows, I2.rows) ), CV_8UC1 );
//	I1.copyTo( stacked(cv::Rect( 0, 0, I1.cols, I1.rows)) );
//	I2.copyTo( stacked(cv::Rect( I1.cols, 0, I2.cols, I2.rows)) );

	return stacked;
}

// stack two images horizontally
cv::Mat DemoOdo::stack_imgsVert( cv::Mat I1, cv::Mat I2 )
{
	cv::Mat stacked = cv::Mat::zeros( cv::Size( std::max(I1.cols, I2.cols), I1.rows + I2.rows ), CV_8UC1 );
//	I1.copyTo( stacked(cv::Rect( 0, 0, I1.cols, I1.rows)) );
//	I2.copyTo( stacked(cv::Rect( 0, I1.rows, I2.cols, I2.rows)) );

	return stacked;
}

// stack quad images
cv::Mat  DemoOdo::stack_imgsQuad( cv::Mat I1_c, cv::Mat I2_c, cv::Mat I1_p, cv::Mat I2_p )
{
	cv::Mat stacked = cv::Mat::zeros( cv::Size( std::max(I1_c.cols+I2_c.cols, I1_p.cols+I2_p.cols), std::max(I1_c.rows + I1_p.rows, I2_c.rows + I2_p.rows) ), CV_8UC1 );
//	I1_c.copyTo( stacked(cv::Rect( 0, 0, I1_c.cols, I1_c.rows)) );
//	I2_c.copyTo( stacked(cv::Rect( I1_c.cols, 0, I2_c.cols, I2_c.rows)) );
//	I1_p.copyTo( stacked(cv::Rect( 0, I1_c.rows, I1_p.cols, I1_p.rows)) );
//	I2_p.copyTo( stacked(cv::Rect( I1_p.cols, I1_c.rows, I2_p.cols, I2_p.rows)) );

	return stacked;
}

void DemoOdo::plotMatch ( cv::Mat M1,  std::vector<Matcher::p_match> matches, int method )
{
	if ( method == 0 )	// flow
	{
		for ( std::vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++)
		{
			cv::line( M1, cv::Point( (int)it->u1p, (int)it->v1p), cv::Point((int)it->u1c,(int)it->v1c), colors[1], 1, 8, 0 );
			cv::circle( M1, cv::Point((int)it->u1c,(int)it->v1c), 1, colors[0], 1, 8, 0 );
		}
	}
	else if ( method == 1 )		// stereo matches
	{
		for ( std::vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++)
		{
			//cv::line( M1, cv::Point( (int)it->u1p, (int)it->v1p), cv::Point((int)it->u1c + M1.cols/2,(int)it->v1c), colors[1], 1, 8, 0 );
			//cv::circle( M1, cv::Point((int)it->u1p,(int)it->v1p), 1, colors[0], 1, 8, 0 );
			//cv::circle( M1, cv::Point((int)it->u1c + M1.cols/2,(int)it->v1c), 1, colors[0], 1, 8, 0 );

			//Horizontal
			cv::line( M1, cv::Point( (int)it->u1c, (int)it->v1c), cv::Point((int)it->u2c + M1.cols/2,(int)it->v2c), cv::Scalar(255,0,0), 2, 8, 0 );
			cv::circle( M1, cv::Point((int)it->u1c,(int)it->v1c), 2, cv::Scalar(0,0,255), 4, 8, 0 );
			cv::circle( M1, cv::Point((int)it->u2c + M1.cols/2,(int)it->v2c), 2, cv::Scalar(0,0,255), 4, 8, 0 );

			////Vertical
			//cv::line( M1, cv::Point( (int)it->u1c, (int)it->v1c), cv::Point((int)it->u2c,(int)it->v2c+M1.rows/2), cv::Scalar(255,0,0), 1, 8, 0 );
			//cv::circle( M1, cv::Point((int)it->u1c,(int)it->v1c), 2,  cv::Scalar(0,0,255), 1, 8, 0 );
			//cv::circle( M1, cv::Point((int)it->u2c,(int)it->v2c+M1.rows/2), 2,  cv::Scalar(0,0,255), 1, 8, 0 );
		}
	}
	else
	{}
}

void DemoOdo::plotMatch( cv::Mat M1,  std::vector<Matcher::p_match> matches, std::vector<int32_t> inliers, int method )
{
	if ( method == 0 )	// flow
	{
		int i = 0;
		int j = 0;
		for ( std::vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++)
		{
			if( inliers[j] == i)
			{
				cv::line( M1, cv::Point( (int)it->u1p, (int)it->v1p), cv::Point((int)it->u1c,(int)it->v1c), cv::Scalar(255,0,0), 1, 8, 0 );
				cv::circle( M1, cv::Point((int)it->u1c,(int)it->v1c), 1, cv::Scalar(255,0,0), 1, 8, 0 );
				j++;
			}
			else
			{
				cv::line( M1, cv::Point( (int)it->u1p, (int)it->v1p), cv::Point((int)it->u1c,(int)it->v1c), cv::Scalar(0,0,255), 1, 8, 0 );
				cv::circle( M1, cv::Point((int)it->u1c,(int)it->v1c), 1, cv::Scalar(0,0,255), 1, 8, 0 );
			}
			i++;
		}
	}
	else if ( method == 1 )		// stereo matches
	{
		for ( std::vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++)
		{
			//cv::line( M1, cv::Point( (int)it->u1p, (int)it->v1p), cv::Point((int)it->u1c + M1.cols/2,(int)it->v1c), colors[1], 1, 8, 0 );
			//cv::circle( M1, cv::Point((int)it->u1p,(int)it->v1p), 1, colors[0], 1, 8, 0 );
			//cv::circle( M1, cv::Point((int)it->u1c + M1.cols/2,(int)it->v1c), 1, colors[0], 1, 8, 0 );

			//Horizontal
			cv::line( M1, cv::Point( (int)it->u1c, (int)it->v1c), cv::Point((int)it->u2c + M1.cols/2,(int)it->v2c), cv::Scalar(255,0,0), 2, 8, 0 );
			cv::circle( M1, cv::Point((int)it->u1c,(int)it->v1c), 2, cv::Scalar(0,0,255), 4, 8, 0 );
			cv::circle( M1, cv::Point((int)it->u2c + M1.cols/2,(int)it->v2c), 2, cv::Scalar(0,0,255), 4, 8, 0 );

			////Vertical
			//cv::line( M1, cv::Point( (int)it->u1c, (int)it->v1c), cv::Point((int)it->u2c,(int)it->v2c+M1.rows/2), cv::Scalar(255,0,0), 1, 8, 0 );
			//cv::circle( M1, cv::Point((int)it->u1c,(int)it->v1c), 2,  cv::Scalar(0,0,255), 1, 8, 0 );
			//cv::circle( M1, cv::Point((int)it->u2c,(int)it->v2c+M1.rows/2), 2,  cv::Scalar(0,0,255), 1, 8, 0 );
		}
	}
	else
	{}
}

void DemoOdo::plotCoordinate( cv::Mat M_Coordinate,
								const cv::Point point_center,	// coordinate center in image frame
								const double ratioW_draw,		// ratioW_draw = pixels/meter along witdth
								const double ratioH_draw, 		// ratioH_draw = pixels/meter along heigth
								const float range_x,					// range of x-axis
								const float range_z)					// range of z-axis
{
	//// set the background
	//cv::Mat roi(M_Coordinate, cv::Rect(0,0,M_Coordinate.cols, M_Coordinate.rows) );	// select a ROI
	//roi = cv::Scalar(255,0,0);							// fill the ROI with (0,0,0) (which is black in BGR space);

	// draw coordinate
	cv::line( M_Coordinate, cv::Point(0,M_Coordinate.cols/2), cv::Point(M_Coordinate.rows,M_Coordinate.cols/2), cv::Scalar(0,255,0),1,8,0);
	cv::line( M_Coordinate, cv::Point(M_Coordinate.rows/2,0), cv::Point(M_Coordinate.rows/2,M_Coordinate.cols),cv::Scalar(0,255,0),1,8,0);
}

// Matrix Tr
//
//	^ z					-------------> u
//	|					|
//	|					|
//	|					|
//  ----------> x		+ v
void DemoOdo::plotTrajectory( cv::Mat I,
								const Matrix Tr,
								const cv::Point point_center,	// coordinate center in image frame
								const double ratioW_draw,		// ratioW_draw = pixels/meter along witdth
								const double ratioH_draw,		// ratioH_draw = pixels/meter along heigth)
								const float range_x,
								const float range_z)
{
	int u = (int)(Tr.val[0][3]*ratioW_draw)+ point_center.x;	// u along row
	int v = point_center.y-(int)(Tr.val[2][3]*ratioH_draw);		// v along col
	cv::circle( I, cv::Point( u, v ), 1, cv::Scalar(0,0,255), 1, 8, 0 );

	std::stringstream ss1, ss2;
	ss1 << range_x;
	ss2 << range_z;
	std::string text = "z_range " + ss1.str() + " x_range " + ss2.str();
	// center the text
	cv::Point textOrg(50, 50);
	int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 1;
	int thickness = 1;
	cv::putText(I, text, textOrg, fontFace, fontScale,
        cv::Scalar::all(255), thickness, 8);

	// draw robot oritation
	cv::Mat R_matrix = (cv::Mat_<double>(3,3) << Tr.val[0][0], Tr.val[0][1], Tr.val[0][2],
												 Tr.val[1][0], Tr.val[1][1], Tr.val[1][2],
												 Tr.val[2][0], Tr.val[2][1], Tr.val[2][2]);
	cv::Mat T_vector = (cv::Mat_<double>(3,1) << Tr.val[0][3], Tr.val[1][3], Tr.val[2][3]);
	cv::Mat r_vector;
	cv::Rodrigues(R_matrix, r_vector);
	cv::Point2d pt;
	//pt.x = u + 0*std::cos(r_vector.at<double>(1)) + 30*std::sin(r_vector.at<double>(1));
	//pt.y = v - 0*std::sin(r_vector.at<double>(1)) + 30*std::cos(r_vector.at<double>(1));
	//	cv::line ( I, cv::Point(u,v), pt, cv::Scalar( 255, 255, 255), 1, 8, 0);

//	std::cout << "r\n" <<  r_vector << std::endl;
//	std::cout << "T\n" <<  T_vector << std::endl;
}

// odometry
//
//					^ x					-------------> u
//					|					|
//					|					|
//					|					|
// y <---------------					+ v
void DemoOdo::plotTrajectory( cv::Mat I,
								const Pose odometry,
								const cv::Point point_center,	// coordinate center in image frame
								const double ratioW_draw,		// ratioW_draw = pixels/meter along witdth
								const double ratioH_draw,		// ratioH_draw = pixels/meter along heigth)
								const float range_x,
								const float range_z)
{
	int u = point_center.x - (int)(odometry.p.y*ratioW_draw);	// u along row
	int v = point_center.y-(int)(odometry.p.x*ratioH_draw);		// v along col
	cv::circle( I, cv::Point( u, v ), 1, cv::Scalar(255,0,0), 1, 8, 0 );

	std::stringstream ss1, ss2;
	ss1 << range_x;
	ss2 << range_z;
	std::string text = "z_range " + ss1.str() + " x_range " + ss2.str();
	// center the text
	cv::Point textOrg(50, 50);
	int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 1;
	int thickness = 1;
	cv::putText(I, text, textOrg, fontFace, fontScale,
        cv::Scalar::all(255), thickness, 8);

	// draw robot oritation
	cv::Point2f dpt;
	dpt.x = odometry.p.x + 1*cos(odometry.phi);
	dpt.y = odometry.p.y + 1*sin(odometry.phi);
	int du = (int)(dpt.y*ratioW_draw);					// u along row
	int dv = -(int)(dpt.x*ratioH_draw);		// v along col
	//pt.x = u + 0*std::cos(r_vector.at<double>(1)) + 30*std::sin(r_vector.at<double>(1));
	//pt.y = v - 0*std::sin(r_vector.at<double>(1)) + 30*std::cos(r_vector.at<double>(1));
	//cv::line(I, cv::Point(u,v), cv::Point(u+du,v+dv),cv::Scalar( 255, 0, 0),1,8,0);
	cv::circle( I, cv::Point( u+du,v+dv ), 1, cv::Scalar(255,0,0), 1, 8, 0 );
}

void DemoOdo::plotTrajectory( cv::Mat I,
							const std::vector<Pose> Odometries,
							const cv::Point point_center,	// coordinate center in image frame
							const double ratioW_draw,		// ratioW_draw = pixels/meter along witdth
							const double ratioH_draw,		// ratioH_draw = pixels/meter along heigth
							const float range_x,
							const float range_z)
{
	for(int i = 0; i < Odometries.size(); i++ )
	{
		int u = point_center.x - (int)(Odometries[i].p.y*ratioW_draw + 0.5);	// u along row
		int v = point_center.y - (int)(Odometries[i].p.x*ratioH_draw + 0.5);		// v along col
		cv::circle( I, cv::Point( u, v ), 1, cv::Scalar(255,0,0), 1, 8, 0 );
		if( i == Odometries.size()-1 )	// draw the current orientation
		{
			int u1 = point_center.x - (int)( (Odometries[i].p.y+1*sin(Odometries[i].phi))*ratioW_draw + 0.5);	// u along row
			int v1 = point_center.y - (int)( (Odometries[i].p.x+1*cos(Odometries[i].phi))*ratioH_draw + 0.5);		// v along col
			cv::line(I, cv::Point(u,v), cv::Point(u1,v1),cv::Scalar( 0, 0, 255),1,8,0);
		}
	}

	std::stringstream ss1, ss2;
	ss1 << range_x;
	ss2 << range_z;
	std::string text = "z_range " + ss1.str() + " x_range " + ss2.str();
	// center the text
	cv::Point textOrg(50, 50);
	int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 1;
	int thickness = 1;
	cv::putText(I, text, textOrg, fontFace, fontScale,
        cv::Scalar::all(255), thickness, 8);
}

void DemoOdo::plotFeaturePointsIn3D( cv::Mat I1, const std::vector<Matcher::p_match> matches, std::vector<int32_t> inliers, const Matrix Tr , const cv::Point point_center,
							const double ratioW_draw, const double ratioH_draw, const float range_x, const float range_z)
{
	// draw inlier points reprojected in 3D space
	for(int i=0; i < inliers.size(); i++)
	{
		// triangular to the current camera frame
		cv::Point3f point_c, point_l;
		float d = std::max(matches[inliers[i]].u1c - matches[inliers[i]].u2c,(float)1.0);
		point_c.x    = (matches[inliers[i]].u1c - calib_cu)*calib_b/d;
		point_c.y    = (matches[inliers[i]].v1c - calib_cv)*calib_b/d;
		point_c.z    = calib_f*calib_b/d;

		// translate to the local map
		point_l.x = Tr.val[0][0]*point_c.x + Tr.val[0][1]*point_c.y + Tr.val[0][2]*point_c.z + Tr.val[0][3];
		point_l.y = Tr.val[1][0]*point_c.x + Tr.val[1][1]*point_c.y + Tr.val[1][2]*point_c.z + Tr.val[1][3];
		point_l.z = Tr.val[2][0]*point_c.x + Tr.val[2][1]*point_c.y + Tr.val[2][2]*point_c.z + Tr.val[2][3];

		int u = (int)(point_l.x*ratioW_draw)+ point_center.x;		// u along row
		int v = point_center.y-(int)(point_l.z*ratioH_draw);		// v along col

		cv::line( I1, cv::Point( u, v-3 ), cv::Point( u, v+3 ), cv::Scalar(255,0,0), 1, 8, 0);
		cv::line( I1, cv::Point( u-3, v ), cv::Point( u+3, v ), cv::Scalar(255,0,0), 1, 8, 0);
	}
}



