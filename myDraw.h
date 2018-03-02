/*
 * myDraw.h
 *
 *  Created on: Oct 30, 2012
 *      Author: robot
 */

#ifndef MYDRAW_H_
#define MYDRAW_H_

#include <opencv2/core/core.hpp>
#include <set>
#include "RobotLink.h"
#include <visp/vpImage.h>
#include "myUtilities.h"

class MyDraw
{
public:
	MyDraw();

	/*
	 *  set and get
	 */
	int		maximumRadiusGet();
	void	maximumRadiusSet(int maximum_radius);
	int		scanAngleGet();
	void	scanAnglSet(int scan_angle);
	int		stepNumberGet();
	void	stepNumberSet(int step_number);
	double	rangeScaleGet();
	void	rangeScaleSet(double range_scale);
	double	angularResolutionGet();
	void	angularResolutionSet(double angular_resolution);

	int		radiusNaviGet();
	void	radiusNaviSet(int radius_navi);

	/*
	 *  laser sensor
	 */
	// draw navigation coordinate
	void drawLaserCoordinate(const float width_area, const float height_area, cv::Mat &I);
	// draw raw measurements of laser scane
	void drawLaser(const LaserScan *las, const float width_area, const float height_area, cv::Mat &I);
	// draw noise filtered of laser scan
	void drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<int> status, cv::Mat &I);
	// draw cluster
	void drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, cv::Mat &I);
	// draw Polygonal Chains after line fitting
	void drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, const std::vector<std::set<int> > PolygonalChains, cv::Mat &I);
	// draw only Polygonal Chains after line fitting
	void drawPolygonalChains(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, const std::vector<std::set<int> > PolygonalChains, cv::Mat &I);
	// draw only Convex Chains after line fitting
	void drawConvexChains(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, const std::vector<std::set<int> > ConvexChains, cv::Mat &I);

	//// draw with given color
	// draw cluster
	void drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, const cv::Scalar &color, cv::Mat &I);
	// draw only Polygonal Chains after line fitting
	void drawPolygonalChains(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, const std::vector<std::set<int> > PolygonalChains, const cv::Scalar &color, cv::Mat &I);
	// draw only Convex Chains after line fitting
	void drawConvexChains(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, const std::vector<std::set<int> > ConvexChains, const cv::Scalar &color, cv::Mat &I);

	//// draw with random colors
	void drawLaserResults( const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Segments,
							const std::vector<std::vector<int> > Clusters, const std::vector<std::set<int> > PolygonalChains,
							const std::vector<std::set<int> > ConvexChains, cv::Mat &I_laserSegments,
							cv::Mat &I_laserClusteringAndMerging, cv::Mat &I_laserPolygonalChains, cv::Mat &I_laserConvexChains);

	// draw laser results based on opencv function
	void drawContourCurves(const LaserScan *las, const float width_area, const float height_area, const std::vector< std::vector<cv::Point2f> > &contourCurves, cv::Mat &I);
	void drawPolygonalCurves(const LaserScan *las, const float width_area, const float height_area, const std::vector< std::vector<cv::Point2f> > &polygonalCurves, cv::Mat &I);
	void drawConvexCurves(const LaserScan *las, const float width_area, const float height_area, const std::vector< std::vector<cv::Point2f> > &convexCurves, cv::Mat &I);

	/**
	 * draw obstacle avoidance
	 */
	// draw reactive obstacle
	void drawObstacleReactive( const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters,
							   const std::vector<std::set<int> > PolygonalChains, const bool b_avoid, const int index_avoid, cv::Mat &I);
	// draw B-spline
	void drawBSpline(const float width_area, const float height_area, const std::vector<Pose> curve_BSpline, cv::Mat &I);
	// draw nearest point
	void drawNearestPoint(const LaserScan *las, const float width_area, const float height_area, const cv::Point2f ptObsNear, cv::Mat &I);
	// predicted trajectory
	int drawPredictedTrajecotry( const std::vector<cv::Point2f> &trajectoryPrediction,	// predicted trajecotry in meter
								 const float scale_w,									// scale from the trajectory coordiantes to draw window
								 const float scale_h,
								 const float robot_R,									// radius of the robot
								 const bool b_avoid,
								 cv::Mat &I);

	/**
	 *  visual object
	 */
	static void drawTargetFourCorners(const std::vector<cv::Point2f> obj_corners, vpImage<vpRGBa> &I);
	void drawFeatureEstimation(const std::vector<cv::Point2f> &pts, const cv::Point2f g, cv::Mat &I);


	// draw target direction in the robot frame
	void drawTargetDirection( const float theta,	// direction
							  const float r,		// range of the direction line
							  const float scale_w,
							  const float scale_h,
							  cv::Mat &I);


	/*
	 *  Utilities
	 */
	// transform laser scan into cartesian coordinate, same as robot coordinate (in millimeter)
	void scan2Cartesian(const LaserScan *las, int index, cv::Point2f &pt);	// in millimeter
	void cartesian2DrawWindows(const cv::Point2f pt1, const float scale_w, const float scale_h,
									const int width_win, const int height_win, cv::Point2i &pt2);

//private:
	/*
	 *  laser sensor
	 */
	int		_maximum_radius;		// in millimeter	4095 for our laser
	int		_scan_angle;			// _step_number*_angular_resolution
	int		_step_number;
	double	_range_scale;			// with respect to 1 meter: 1000 for our laser
	double	_angular_resolution;	// in Radian

	float   _wScale_meter2laserwindow;	// the scale of unit from meter to laser window
	float 	_hScale_meter2laserwindow;	// the scale of unit from meter to laser window

	/*
	 *	navigation window
	 */
	int		_radius_navi;				// the desired range to be drawn on the image window in millimeter
	double 	_wScale_navi;				// the scale of image pixel with respect to millimeter along the width axis of navigation demo window
	double 	_hScale_navi;				// the scale of image pixel with respect to millimeter along the height axis of navigation demo window
};


#endif /* MYDRAW_H_ */
