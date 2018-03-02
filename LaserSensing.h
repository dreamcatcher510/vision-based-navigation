/*
 * LaserSensing.h
 *
 *  Created on: Oct 30, 2012
 *      Author: robot
 */

#ifndef LASERSENSING_H_
#define LASERSENSING_H_

#include <set>
#include "RobotLink.h"
#include "BSpline.h"

class LaserSensing
{
public:
	LaserSensing();
	LaserSensing(int _maximum_radius, int _step_number, double range_scale, double angular_resolution);
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

	/*
	 * obstacle detection using laser range finder
	 */
	void obstacleDetection(const LaserScan * las,
							std::vector<int> &status,						//	status for noise filtering
							std::vector<std::vector<int> >  &Segments,		//  segments
							std::vector<std::vector<int> >  &Clusters,		//	clusters after merging
							std::vector<std::set<int> >  &PolygonalChains,	//	line fitting
							std::vector<std::set<int> >  &ConvexChains);
	void laserNoiseFilter(const LaserScan * las,
							const float threshold,						// threshold to identify big jumps using adjacent range values
							std::vector<int> &status);					// label scans good 1 or not 0
	// laser clustering/segmentation
	// it divides the filtered points into groups of close points and discards groups consisting of too few points
	void laserCluster(const LaserScan *las,
						const std::vector<int> status,					// noise filtered status 1 good 0 noise
						const short threshold,							// threshold for clustering scans/segmentation
						const short length_minimum,						// the minimum number of scans to be regarded as a cluster
						std::vector< std::vector<int> > &Clusters);		// label the segmented clusters

	// laser clustering/segmentation
	// it divides the filtered points into groups of close points and discards groups consisting of too few points
	void laserSegment(const LaserScan *las,
						const std::vector<int> status,					// noise filtered status 1 good 0 noise
						const short threshold,							// threshold for clustering scans/segmentation
						const short length_minimum,						// the minimum number of scans to be regarded as a cluster
						std::vector< std::vector<int> > &Segments);		// label the segmentations

	// computer the the Cartesian distance of the adjacent end points between two adjacent clusters
	void laserClusterMerge(const LaserScan *las,
							const float threshold,						//
							std::vector< std::vector<int> > &Clusters);

	// combine Polar coordinate system and Cartisian coordinate
	void laserClusterMerge(const LaserScan *las,
							const int	threshold_index,
							const float threshold_c,
							const float threshold_p,
							std::vector< std::vector<int> > &Clusters);

	void LaserRecursionCalcPolygonalChain( const LaserScan *las,
											const std::vector<int> cluster,
											const int index_ini , 				// search start index of cluster
	                                        const int index_end, 				// search end index of cluster
	                                        const float threshold,
	                                        std::set<int> &polygonalchain);

	void LaserRecursionCalcConvexHull( const LaserScan *las,
										const std::vector<int> cluster,
										const int index_ini , 				// search start index of cluster
										const int index_end, 				// search end index of cluster
										const float threshold,
										std::set<int> &convexchain);

	// find an open polygonal chain to each cluster
	void laserLineFitting(const LaserScan *las,
							const float threshold,
							const std::vector<std::vector<int> > Clusters,
							std::vector<std::set <int> > &PolygonalChains);	 // polygonalchains restore the index of las->data

	// find an open convex hull to each cluster
	void laserconvexhull(const LaserScan *las,
							const float threshold,
							const std::vector<std::vector<int> > Clusters,
							std::vector<std::set <int> > &ConvexChains);	 // polygonalchains restore the index of las->data

	// obstacle detection based on opencv function
	void obstacleDetection( const LaserScan * las,
							std::vector<int> &status,									//	status for noise filtering
							std::vector<std::vector<int> >  &Clusters,					//	clusters after merging
							std::vector<std::vector<cv::Point2f> > 	  &contourCurves,	// contour curves
							std::vector< std::vector <cv::Point2f> >  &polygonalCurves,	//	polygonal curves
							std::vector< std::vector <cv::Point2f> >  &convexCurves);	//	convex curves

	// Updata reactive obstacle and the relative odometry data
	void previousReactiveObstacleUpdate( const LaserScan * las,								// raw laser data
			 	 	 	 	 	 	 	 const int indexClusters_reactive,					// which obstacle to react
										 const std::vector<std::set <int> > &ConvexChains,	//	clusters after merging
										 const Pose &odometry,								// odometry data
										 std::vector<cv::Point2f> &obstaclePrevious,
										 Pose &odometryPrevious );							// odometry relative to the remembered previous obstacle

	// previous reactive obstacle tracking
	void reactiveObstacleTrack( const LaserScan * las,								// raw laser data
								 const std::vector<std::set <int> > &ConvexChains,	// clusters after merging
								 const Pose &odometry,								// odometry data
								 const std::vector<cv::Point2f> &obstaclePrevious,
								 const Pose &odometryPrevious,
								 const float threshold,								// tracking accuracy
								 int &indexClusters_reactive);						// which obstacle to react

	/*
	 *  Utilities
	 */
	// transform laser scan into cartesian coordinate, same as robot coordinate (in millimeter)
	void scan2Cartesian(const LaserScan *las, int index, cv::Point2f &pt);	// in millimeter
	void scan2CartesianInMeter(const LaserScan *las, int index, cv::Point2f &pt);	// in meter

	/**
	 *  detection results
	 */
	// using my method
	std::vector<int> status;
	std::vector<std::vector<int> >  _Segments;
	std::vector<std::vector<int> >  Clusters;
	std::vector<std::set<int> >  PolygonalChains;
	std::vector<std::set<int> >  ConvexChains;
	BSplineCubic bspline;						// B-spline approximation
	std::vector<cv::Point2f> _obstaclePrevious;	// previous obstacle
	Pose _odometryPrevous;						// odometry relative to the remembered previous obstacle


	// using opencv libraries
	std::vector< std::vector <cv::Point2f> >  contourCurves;	// contour curves
	std::vector< std::vector <cv::Point2f> >  polygonalCurves;	// polygonal curves
	std::vector< std::vector <cv::Point2f> >  convexCurves;		// convex curves

private:
	int		_maximum_radius;		// in millimeter	4000 for our laser
	int		_scan_angle;			// _step_number*_angular_resolution
	int		_step_number;
	double	_range_scale;			// with respect to 1 meter: 1000 for our laser
	double	_angular_resolution;	// in Radian


};


#endif /* LASERSENSING_H_ */
