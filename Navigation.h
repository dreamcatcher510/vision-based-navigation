#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

//#include <list>
#include <set>
#include "RobotLink.h"
#include "myUtilities.h"
#include "matcher.h"
#include "visualodometry.h"

class Navigation
{
public:
	Navigation();

	// initialization
	bool initLaser(const float scaleLaser, const float angularResolution, const short maximumRadius);
	bool initLaser(const float scaleLaser, const float angularResolution, const short maximumRadius, const cv::Mat I);

	// draw laser data
	bool drawLaserCoordinate(const float width_area, const float height_area, cv::Mat &I);
	bool drawLaser(const LaserScan *las, const float width_area, const float height_area, cv::Mat &I);
	bool drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<int> status, cv::Mat &I);
	bool drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int>* > Clusters, cv::Mat &I);
	bool drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, cv::Mat &I);
	bool drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters, const std::vector<std::set<int> > PolygonalChains, cv::Mat &I);
	bool drawObstacleReactive(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters,
								const std::vector<std::set<int> > PolygonalChains, const bool b_avoid, const int index_avoid, cv::Mat &I);
	bool drawNearestPoint(const LaserScan *las, const float width_area, const float height_area, const cv::Point2f ptObsNear, cv::Mat &I);

	bool drawBSpline(const float width_area, const float height_area, const std::vector<Pose> curve_BSpline, cv::Mat &I);

	// draw image
	int drawFeatureEstimation(const std::vector<cv::Point2f> &pts,
								const int state,
								cv::Mat &I_1,
								cv::Mat &I_track,
								cv::Mat &I);

	bool scan2Cartesian(const LaserScan *las, int index, cv::Point2f &pt);
	void scan2CartesianInMeter(const LaserScan *las, int index, cv::Point2f &pt);
	bool cartesian2DrawWindows(const cv::Point2f pt1, const float scale_w, const float scale_h,
								const int width_win, const int height_win, cv::Point2i &pt2);

	// laser data processing
	int laserNoiseFilter(const LaserScan * las, const float threshold, std::vector<int> &status);	// laser data filtering: reject isolated points
	int laserCluster(const LaserScan *las, const std::vector<int> status,
					const short threshold,						// the minimum distance between two nearby clusters
					const short length_minimum,					// the minimum length of a cluster
					std::vector<std::vector<int>* > &Clusters); // laser data segmentation/clustering
	int laserCluster(const LaserScan *las, const std::vector<int> status,
						const short threshold,						// the minimum distance between two nearby clusters
						const short length_minimum,					// the minimum length of a cluster
						std::vector<std::vector<int> > &Clusters); // laser data segmentation/clustering
	int laserClusterMerge(const LaserScan *las, const float threshold, std::vector<std::vector<int>* > &Clusters);	// merge the adjacent clusters
	int laserClusterMerge(const LaserScan *las, const float threshold, std::vector<std::vector<int> > &Clusters);	// merge the adjacent clusters
	int LaserRecursionCalcPolygonalChain( const LaserScan *las,
											const std::vector<int> cluster,
											const int index_ini , 			// line initial point index of cluster
	                                        const int index_end,  			// line end point index of cluster
	                                        const float threshold,
	                                        std::set<int> &polygonalchain);
	int laserLineFitting(const LaserScan *las, const float threshold, const std::vector<std::vector<int> > Clusters,
							std::vector<std::set<int> > &PolygonalChains);		 // find an open polygonal chain to each cluster

	int stereoOdometry(const cv::Mat M1, const cv::Mat M2, Matcher *M, VisualOdometry *VO);

	int reactiveObstacle();

	int deleteClusters(std::vector<std::vector<int>* > &Clusters);

	float getScaleLaser();

private:
	float _scaleLaser;	// respect to 1 meter
	float _angularResolution;
	short _maximumRadius;
	float _scale_w;		// the width of navigation demo window
	float _scale_h;		// the width of navigation demo window

};


#endif
