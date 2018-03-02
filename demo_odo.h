#ifndef DEMOODO_H
#define DEMOODO_H

#include <string>
#include <opencv2/opencv.hpp>
#include "matcher.h"
#include "matcherMex.h"
#include "myUtilities.h"

class DemoOdo{
public:
	int init(std::string m, std::string f);
	int initCalib(double f, double cu, double cv, double base);
	int runStereo_rectifiedInput();

	int stereoRectification(const cv::Mat cameraMatrix1, const cv::Mat distCoeffs1, const cv::Mat cameraMatrix2, const cv::Mat distCoeffs2,
						const cv::Size imageSize, const cv::Mat R, const cv::Mat T, cv::Mat &R1, cv::Mat &R2, cv::Mat &P1, cv::Mat &P2, cv::Mat &Q,
						const cv::Size newImageSize, cv::Rect &validPixROI1, cv::Rect &validPixROI2, cv::Mat rmap[][2]);

	int stereoOdometry(const cv::Mat M1, const cv::Mat M2,
						MatcherMex &matcherMex,
						bool &success, int &failure);

	cv::Mat stack_imgsHori( cv::Mat I1, cv::Mat I2 );	// stack two images
	cv::Mat stack_imgsVert( cv::Mat I1, cv::Mat I2 );	// stack two images
	cv::Mat stack_imgsQuad( cv::Mat I1_c, cv::Mat I2_c, cv::Mat I1_p, cv::Mat I2_p );	// stack quad images
	void plotMatch( cv::Mat M1,  std::vector<Matcher::p_match> matches, int method );
	void plotMatch( cv::Mat M1,  std::vector<Matcher::p_match> matches, std::vector<int32_t> inliers, int method );
	void plotCoordinate( cv::Mat M_Coordinate,
						const cv::Point point_center,	// coordinate center in image frame
						const double ratioW_draw,		// ratioW_draw = pixels/meter along witdth
						const double ratioH_draw,		// ratioH_draw = pixels/meter along heigth
						const float range_x,
						const float range_z);
	void plotTrajectory( cv::Mat I1, const Matrix Tr,
								const cv::Point point_center,	// coordinate center in image frame
								const double ratioW_draw,		// ratioW_draw = pixels/meter along witdth
								const double ratioH_draw,		// ratioH_draw = pixels/meter along heigth
								const float range_x,
								const float range_z);

	void plotTrajectory( cv::Mat I, Pose odometry,
								const cv::Point point_center,	// coordinate center in image frame
								const double ratioW_draw,		// ratioW_draw = pixels/meter along witdth
								const double ratioH_draw,		// ratioH_draw = pixels/meter along heigth
								const float range_x,
								const float range_z);

	void plotTrajectory( cv::Mat I,
								const std::vector<Pose> Odometries,
								const cv::Point point_center,	// coordinate center in image frame
								const double ratioW_draw,		// ratioW_draw = pixels/meter along witdth
								const double ratioH_draw,		// ratioH_draw = pixels/meter along heigth
								const float range_x,
								const float range_z);

	void plotFeaturePointsIn3D( cv::Mat I1, const std::vector<Matcher::p_match> matches, std::vector<int32_t> inliers, const Matrix Tr , const cv::Point point_center,
							const double ratioW_draw, const double ratioH_draw, const float range_x, const float range_z);

	std::string getMethod() { return method; };
	std::string getFolder() { return files_folder; };
private:
	std::string files_folder;
	std::string method;
	cv::Scalar colors[10];

	double calib_f;
	double calib_cu;
	double calib_cv;
	double calib_b;
};

#endif // DEMOODO_H
