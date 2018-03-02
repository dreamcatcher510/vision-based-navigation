/*
 * LaserSensing.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: robot
 */

#include "LaserSensing.h"
#include "myUtilities.h"
#include <opencv2/imgproc/imgproc.hpp>

LaserSensing::LaserSensing()
{
}

LaserSensing::LaserSensing(int maximum_radius, int step_number, double range_scale, double angular_resolution)
{
	_angular_resolution = angular_resolution;
	_maximum_radius = maximum_radius;
	_range_scale = range_scale;
	_scan_angle = angular_resolution*step_number;
	_step_number = step_number;
}

double LaserSensing::angularResolutionGet()
{
	return _angular_resolution;
}

void LaserSensing::angularResolutionSet(double angular_resolution)
{
	_angular_resolution = angular_resolution;
}

int LaserSensing::maximumRadiusGet()
{
	return _maximum_radius;
}

void LaserSensing::maximumRadiusSet(int maximum_radius)
{
	_maximum_radius = maximum_radius;
}

double LaserSensing::rangeScaleGet()
{
	return _range_scale;
}

void LaserSensing::rangeScaleSet(double range_scale)
{
	_range_scale = range_scale;
}

int LaserSensing::scanAngleGet()
{
	return _scan_angle;
}

void LaserSensing::scanAnglSet(int scan_angle)
{
	_scan_angle = scan_angle;
}

int LaserSensing::stepNumberGet()
{
	return _step_number;
}

void LaserSensing::stepNumberSet(int step_number)
{
	_step_number = step_number;
}

// obstacle detection based on opencv function
void LaserSensing::obstacleDetection( const LaserScan * las,
									  std::vector<int> &status,										// status for noise filtering
									  std::vector<std::vector<int> >  &Clusters,					// clusters after merging
									  std::vector< std::vector <cv::Point2f> > &contourCurves,		// contour curves
									  std::vector< std::vector <cv::Point2f> > &polygonalCurves,	// polygonal curves
									  std::vector< std::vector <cv::Point2f> > &convexCurves)		// convex curves
{
	/*
	 * 	 step 1: laser data filtering: reject isolated point
	 */
	float threshold_noise = 100;	// 100 mm = 0.1 m
	laserNoiseFilter(las,threshold_noise, status);

	/*
	 *	step 2: laser clustering/segmentation
	 */
	short threshold_clusting = 200;	// the minimum distance between two nearby clusters
	short length_min = 4;			// the minimum length of a cluster, discards groups consisting of too few points
	laserCluster(las,status, threshold_clusting, length_min, Clusters);

	/*
	 *  step 3: data association
	 */
	float threshold_merge = 200;
	// method one: directly cartisian distance
	//laserClusterMerge( las, threshold_merge, Clusters );
	// method two: combine Polar coordinate system and cartisian coordinate
	int		threshold_index = 5;
	float	threshold_c = 200;
	float	threshold_p = 600;
	laserClusterMerge( las, threshold_index, threshold_c, threshold_p, Clusters );

	/*
	 *  step 4: generate contour curves in the robot frame
	 */
	contourCurves.clear();
	std::vector< std::vector<int> >::iterator iter_Clusters;
	for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end(); iter_Clusters++ )
	{
		std::vector<int>::iterator iter_cluster;
		std::vector<cv::Point2f> contourCurve;
		for( iter_cluster = iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++)
		{
			cv::Point2f pt;
			scan2Cartesian(las, *iter_cluster, pt);
			contourCurve.push_back(pt);
		}
		contourCurves.push_back(contourCurve);
	}

	/*
	 *  step 5: calculate the polygonal curves using split-and-merge method
	 */
	polygonalCurves.clear();
	double threshold_polygonalCurve = 100;	// 100 mm
	bool closed = false;
	for( int i = 0; i < contourCurves.size(); i++ )
	{
		std::vector<cv::Point2f> polygonalCurve;
		cv::approxPolyDP(contourCurves[i], polygonalCurve, threshold_polygonalCurve, closed);
		polygonalCurves.push_back(polygonalCurve);
	}

	/*
	 *  step 6: calculate the convex curves
	 */
	// use contour curves
	convexCurves.clear();
//	for( int i = 0; i < contourCurves.size(); i++ )
//	{
//		std::vector<cv::Point2f> convexCurve;
//		cv::convexHull( contourCurves[i], convexCurve, false );
//		convexCurves.push_back(convexCurve);
//	}
	// use polygonal curves
	// note that convexHull calculate convex hull, not convex curve. This is not our case
	for( int i = 0; i < polygonalCurves.size(); i++ )
	{
		std::vector<cv::Point2f> convexCurve;
		cv::convexHull( polygonalCurves[i], convexCurve, false );
		convexCurves.push_back(convexCurve);
	}

}

// my function
void LaserSensing::obstacleDetection(const LaserScan * las,
										std::vector<int> &status,
										std::vector<std::vector<int> >  &Segments,					// clusters after merging
										std::vector<std::vector<int> >  &Clusters,
										std::vector<std::set<int> >  &PolygonalChains,
										std::vector<std::set<int> >  &ConvexChains)
{
	/*
	 * 	 step 1: laser data filtering: reject isolated point
	 */
	float threshold_noise = 100;	// 100 mm = 0.1 m
	//status.clear();
	laserNoiseFilter(las,threshold_noise, status);

	/*
	 *	step 2: laser clustering/segmentation
	 */
//	int n_Clusters = 200;			// reserve 200 cluster at first
//	Clusters.clear();
//	Clusters.reserve(n_Clusters);
	short threshold_segment = 200;	// the minimum distance between two nearby clusters
	short length_min = 3;			// the minimum length of a cluster, discards groups consisting of too few points
	// laserCluster(las,status, threshold_clusting, length_min, Segments);
	laserSegment( las, status, threshold_segment, length_min, Segments );

	/*
	 *  step 3: data association
	 */
	float threshold_merge = 200;
	// method one: directly cartisian distance
	//laserClusterMerge( las, threshold_merge, Clusters );
	// method two: combine Polar coordinate system and cartisian coordinate
	int		threshold_index = 5;
	float	threshold_c = 200;
	float	threshold_p = 600;
	Clusters.clear();
	Clusters = Segments;
	laserClusterMerge( las, threshold_index, threshold_c, threshold_p, Clusters );

	/*
	 *  step 4: line fitting
	 */
//	PolygonalChains.clear();
//	PolygonalChains.reserve(Clusters.size());
	float threshold_lineSegment = 100; // 100 mm
	laserLineFitting(las, threshold_lineSegment, Clusters, PolygonalChains);
	laserconvexhull(las, threshold_lineSegment, Clusters, ConvexChains);
}

void LaserSensing::laserNoiseFilter(const LaserScan * las, const float threshold, std::vector<int> &status)
{
	status.clear();
	for (int i = 0 ; i < las->size; i++)
	{
		int s = 0;
		float dis_im1, dis_ip1;
		cv::Point2f pt_i, pt_ip1;
		if( i == 0)
		{
//			scan2Cartesian(las, i, pt_i);
//			scan2Cartesian(las, i+1, pt_ip1);
//			dis_ip1 = distance2d(pt_i, pt_ip1);
//			if (dis_ip1 > threshold)
//				s = 0;
//			else
//				s = 1;

			dis_ip1 = std::abs( (float)(las->data[i]-las->data[i+1]) );
			if ( dis_ip1 > threshold)
				s = 0;
			else
				s = 1;

		}
		else if ( i == las->size -1 )
		{
			if (dis_im1 > threshold)
				s = 0;
			else
				s = 1;
		}
		else
		{
//			scan2Cartesian(las, i, pt_i);
//			scan2Cartesian(las, i+1, pt_ip1);
//			dis_ip1 = distance2d(pt_i, pt_ip1);
//			if (dis_im1 > threshold && dis_ip1 > threshold)
//				s = 0;
//			else
//				s = 1;

			dis_ip1 = std::abs( (float)(las->data[i]-las->data[i+1]) );

			if (dis_im1 > threshold && dis_ip1 > threshold)
				s = 0;
			else
				s = 1;
		}

		if ( las->data[i] >= _maximum_radius )	// out of range is also isolated points
			s = 0;

		dis_im1 = dis_ip1;

		status.push_back(s);
	}
}

// laser clustering/segmentation
// it divides the filtered points into groups of close points and discards groups consisting of too few points
void LaserSensing::laserCluster(const LaserScan *las,
								const std::vector<int> status,				// noise filtered status 1 good 0 noise
								const short threshold,						// threshold for clustering scans/segmentation
								const short length_minimum,					// the minimum number of scans to be regarded as a cluster
								std::vector< std::vector<int> > &Clusters)	// label the segmented clusters
{
	int n_Clusters = 200;			// reserve 200 cluster at first
	Clusters.clear();
	Clusters.reserve(n_Clusters);
	short r_im1 = 0;
	short r_i = 0;
	std::vector<int> cluster;
	for (int i = 0; i < las->size; i++)
	{
		r_i = las->data[i];
		if ( !cluster.empty() )    // already start the current cluster, and finding the inside points of the cluster
		{
			if ( std::abs(r_i - r_im1) < threshold && status[i] == 1 )	//	is a close point, here is the continuous scanning
																		//	so using the range value not cartesian
			{
				cluster.push_back(i);
				if (i == las->size - 1)									// the last scan, stop the current cluster
				{
					Clusters.push_back(cluster);
					cluster.clear();
					break;
				}
			}
			else														// a big jump, the end of the current cluster
			{
				if (cluster.size() > length_minimum )					// discards groups consisting of too few points
				{
					Clusters.push_back(cluster);
				}
				cluster.clear();
			}
		}

		if ( cluster.empty() 						// wait to start a new cluster
			&& status[i] == 1 )						// not a noise
		{
			cluster.push_back(i); 					// the left point of the cluster
		}

		r_im1 = r_i;
	}
}

// laser segmentation using Cartisian distance
// it divides the filtered points into groups of close points and discards groups consisting of too few points
void LaserSensing::laserSegment(const LaserScan *las,
								const std::vector<int> status,				// noise filtered status 1 good 0 noise
								const short threshold,						// threshold for clustering scans/segmentation
								const short length_minimum,					// the minimum number of scans to be regarded as a cluster
								std::vector< std::vector<int> > &Segments)	// label the segmented clusters
{
	int n_Segments = 200;			// reserve 200 cluster at first
	Segments.clear();
	Segments.reserve(n_Segments);
	int index_previous = 0;
	cv::Point2f pt_previous, pt_current;
	scan2Cartesian(las, index_previous, pt_previous);
	scan2Cartesian(las, 0,  pt_current);
	float dis;
	std::vector<int> segment;
	segment.clear();
	for (int i = 0; i < las->size; i++)
	{
		scan2Cartesian(las, i,  pt_current);
		if ( !segment.empty() )    // already start the current cluster, and finding the inside points of the cluster
		{
			if ( status[i] == 1 )
			{
				dis = distance2d(pt_previous, pt_current);
				if ( dis < threshold )							// add to the current segment
				{
					segment.push_back(i);
					pt_previous = pt_current;
					if (i == las->size - 1)									// the last scan, stop the current cluster
					{
						Segments.push_back( segment );
						segment.clear();
						break;
					}
				}
				else
				{
					if ( segment.size() > length_minimum )					// discards groups consisting of too few points
					{
						Segments.push_back( segment );
					}
					segment.clear();
				}
			}
		}
		if ( segment.empty() ) 		// segment is empty
		{
			if ( status[i] == 1 )						// not a noise
			{
				segment.push_back(i); 					// the left point of the cluster
				pt_previous = pt_current;
			}
		}
	}
}

// computer the the Cartesian distance of the adjacent end points between two adjacent clusters, which is different with noise filter and cluster
void LaserSensing::laserClusterMerge(const LaserScan *las,
										const float threshold,							// threshold for merging two adjacent clusters
										std::vector< std::vector<int> > &Clusters)
{
	 if ( Clusters.size() > 1)
	{
		std::vector< std::vector<int> >::iterator iter_Clusters;
		for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end() - 1; iter_Clusters++ )
		{
			// index of laser scan
			int index_curr_cluster_last = (*iter_Clusters)[iter_Clusters->size()-1];	// the end point of the current cluster
			int index_next_cluster_first = (*(iter_Clusters+1))[0];						// the start point of the next cluster
			double dis = 0;
			cv::Point2f pt_curr, pt_next;
			scan2Cartesian(las, index_curr_cluster_last, pt_curr);
			scan2Cartesian(las, index_next_cluster_first, pt_next);
			dis = distance2d(pt_curr, pt_next);				// here we use distance between two point instead of using r1 and r2
															// because it is not sure that index_curr_cluster_last
															// and index_next_cluster_first are two adjacent scan
			if( dis < threshold )
			{
				std::vector<int>::iterator iter_clusterNext;
				for ( iter_clusterNext =  (iter_Clusters+1)->begin(); iter_clusterNext != (iter_Clusters+1)->end(); iter_clusterNext++ )
				{
					(*iter_Clusters).push_back(*iter_clusterNext);
				}
				(*(iter_Clusters+1)).clear();
				Clusters.erase(iter_Clusters+1);
				iter_Clusters--;
			}
		}

	}
}

// cluster merge is the same as data association, here we simplify it combine Polar coordinate system and cartisian coordinate
// if delta(theta) > 5, if d < threshold_c merging
// if delta(theta) < 5, if delta(ro) < threshold_p merging
void LaserSensing::laserClusterMerge(const LaserScan *las,
										const int	threshold_index,
										const float threshold_c,							// threshold for merging two adjacent clusters
										const float threshold_p,
										std::vector< std::vector<int> > &Clusters)
{
	 if ( Clusters.size() > 1)
	{
		std::vector< std::vector<int> >::iterator iter_Clusters;
		for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end() - 1; iter_Clusters++ )
		{
			// index of laser scan
			int index_curr_cluster_last = (*iter_Clusters)[iter_Clusters->size()-1];	// the end point of the current cluster
			int index_next_cluster_first = (*(iter_Clusters+1))[0];						// the start point of the next cluster
			int delta_index = index_next_cluster_first - index_curr_cluster_last;
			double dis = 0;
			cv::Point2f pt_curr, pt_next;
			scan2Cartesian(las, index_curr_cluster_last, pt_curr);
			scan2Cartesian(las, index_next_cluster_first, pt_next);
			dis = distance2d(pt_curr, pt_next);				// here we use distance between two point instead of using r1 and r2
															// because it is not sure that index_curr_cluster_last
															// and index_next_cluster_first are two adjacent scan

			if ( delta_index > 5 )
			{
				if( dis < threshold_c )
				{
					std::vector<int>::iterator iter_clusterNext;
					for ( iter_clusterNext =  (iter_Clusters+1)->begin(); iter_clusterNext != (iter_Clusters+1)->end(); iter_clusterNext++ )
					{
						(*iter_Clusters).push_back(*iter_clusterNext);
					}
					(*(iter_Clusters+1)).clear();
					Clusters.erase(iter_Clusters+1);
					iter_Clusters--;
				}
			}
			else
			{
				if( dis < threshold_p )
				{
					std::vector<int>::iterator iter_clusterNext;
					for ( iter_clusterNext =  (iter_Clusters+1)->begin(); iter_clusterNext != (iter_Clusters+1)->end(); iter_clusterNext++ )
					{
						(*iter_Clusters).push_back(*iter_clusterNext);
					}
					(*(iter_Clusters+1)).clear();
					Clusters.erase(iter_Clusters+1);
					iter_Clusters--;
				}
			}
		}

	}
}


void LaserSensing::LaserRecursionCalcPolygonalChain( const LaserScan *las,
										const std::vector<int> cluster,
										const int index_ini , 				// search start index of cluster
                                        const int index_end, 				// search end index of cluster
                                        const float threshold,
                                        std::set<int> &polygonalchain)
{
	float dis;
	float dis_max = 0;
	int i_max_laser = 0;
	int i_max_cluster = 0;
	cv::Point2f pt, pt_ini, pt_end;
	int index;
	index = cluster[index_ini];
	scan2Cartesian(las, index, pt_ini);
	index = cluster[index_end];
	scan2Cartesian(las, index, pt_end);
	// search in the searching area of cluster
	for (int i = index_ini + 1; i < index_end; i++) 	// search area of laser data
	{
		index = cluster[i];
		scan2Cartesian(las, index, pt);
		dis = distancePt2Line(pt, pt_ini, pt_end);
		if (dis > dis_max)
		{
			dis_max = dis;
			i_max_laser = index; 	// the index of laser scan, the polygonalchain restore index of laser scan
			i_max_cluster = i;	 	// the index of cluster, to give the search area in cluster
		}
	}
	if ( dis_max > threshold )
	{
		std::set<int>::iterator iter;
		index = cluster[index_end];
		iter = polygonalchain.find( index );
		polygonalchain.insert(iter, i_max_laser);
		LaserRecursionCalcPolygonalChain( las, cluster, index_ini, i_max_cluster, threshold, polygonalchain);
		LaserRecursionCalcPolygonalChain( las, cluster, i_max_cluster, index_end, threshold, polygonalchain);
	}
}

void LaserSensing::LaserRecursionCalcConvexHull( const LaserScan *las,
										const std::vector<int> cluster,
										const int index_ini , 				// search start index of cluster
                                        const int index_end, 				// search end index of cluster
                                        const float threshold,
                                        std::set<int> &convexchain)
{
	float dis;
	float areaTri;
	float dis_max = 0;
	int i_max_laser = 0;
	int i_max_cluster = 0;
	cv::Point2f pt, pt_ini, pt_end;
	int index;
	index = cluster[index_ini];
	scan2Cartesian(las, index, pt_ini);
	index = cluster[index_end];
	scan2Cartesian(las, index, pt_end);
	// search in the searching area of cluster
	for (int i = index_ini + 1; i < index_end; i++) 	// search area of laser data
	{
		index = cluster[i];
		scan2Cartesian(las, index, pt);
		dis = distancePt2Line(pt, pt_ini, pt_end);
		areaTri = pt_ini.x*(pt.y - pt_end.y) - pt.x*(pt_ini.y-pt_end.y) + pt_end.x*(pt_ini.y-pt.y);
		dis = areaTri > 0 ? dis : -dis;
		if (dis > dis_max)
		{
			dis_max = dis;
			i_max_laser = index; 	// the index of laser scan, the polygonalchain restore index of laser scan
			i_max_cluster = i;	 	// the index of cluster, to give the search area in cluster
		}
	}
	if ( dis_max > threshold )
	{
		std::set<int>::iterator iter;
		index = cluster[index_end];
		iter = convexchain.find( index );
		convexchain.insert(iter, i_max_laser);
		LaserRecursionCalcConvexHull( las, cluster, index_ini, i_max_cluster, threshold, convexchain);
		LaserRecursionCalcConvexHull( las, cluster, i_max_cluster, index_end, threshold, convexchain);
	}
}

// find an open polygonal chain to each cluster
void LaserSensing::laserLineFitting(const LaserScan *las,
									const float threshold,
									const std::vector<std::vector<int> > Clusters,
									std::vector<std::set <int> > &PolygonalChains)	 // polygonalchains restore the index of las->data
{
	PolygonalChains.clear();
	PolygonalChains.reserve(Clusters.size());

	std::vector< std::vector<int> >::const_iterator iter_Clusters;

	for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end(); iter_Clusters++ )
	{
		std::set<int> polygonalchain;
		// initial polygonal chain with the two end point of the cluster
		int index_ini = 0;
		int index_end = (*iter_Clusters).size() - 1;
		polygonalchain.insert( (*iter_Clusters).front() );
		polygonalchain.insert( (*iter_Clusters).back() );
		LaserRecursionCalcPolygonalChain( las, *iter_Clusters, index_ini, index_end, threshold, polygonalchain);
		PolygonalChains.push_back(polygonalchain);
	}
}

// find an open convex hull to each cluster
void LaserSensing::laserconvexhull(const LaserScan *las,
									const float threshold,
									const std::vector<std::vector<int> > Clusters,
									std::vector<std::set <int> > &ConvexChains)	 // polygonalchains restore the index of las->data
{
	ConvexChains.clear();
	ConvexChains.reserve(Clusters.size());
	std::vector< std::vector<int> >::const_iterator iter_Clusters;

	for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end(); iter_Clusters++ )
	{
		std::set<int> convexchain;
		// initial convex chain with the two end point of the cluster
		int index_ini = 0;
		int index_end = (*iter_Clusters).size() - 1;
		convexchain.insert( (*iter_Clusters).front() );
		convexchain.insert( (*iter_Clusters).back() );
		LaserRecursionCalcConvexHull( las, *iter_Clusters, index_ini, index_end, threshold, convexchain);
		ConvexChains.push_back(convexchain);
	}
}

// laser cartesian coordinate, same as robot coordinate (in millimeter)
//
//					^ x
//					|
//					|
//					|
//  y <-------------
void LaserSensing::scan2Cartesian(const LaserScan *las, int index, cv::Point2f &pt)	// in millimeter
{
	double angle_trans = 2*PI/3;
    double theta = angle_trans - index * _angular_resolution;
    theta = pi_to_pi(theta);
    pt.x = las->data[index] * std::cos(theta);
    pt.y = las->data[index] * std::sin(theta);
}

void LaserSensing::scan2CartesianInMeter(const LaserScan *las, int index, cv::Point2f &pt)	// in meter
{
	float angle_trans = 2*PI/3;
	float theta = angle_trans - index * _angular_resolution;
	theta = pi_to_pi(theta);
	pt.x = las->data[index] * std::cos(theta)/(float)(_range_scale);
	pt.y = las->data[index] * std::sin(theta)/(float)(_range_scale);
}

// remember reactive obstacle and the relative odometry data
void LaserSensing::previousReactiveObstacleUpdate( const LaserScan * las,								// raw laser data
													 const int indexClusters_reactive,					// which obstacle to react
													 const std::vector<std::set <int> > &ConvexChains,	//	clusters after merging
													 const Pose &odometry,								// odometry data
													 std::vector<cv::Point2f> &obstaclePrevious,
													 Pose &odometryPrevious )							// odometry relative to the remembered previous obstacle

{
	obstaclePrevious.clear();
	//laserSensing._obstaclePrevious = laserSensing.
	std::vector<std::set<int> >::const_iterator iter_ConvexChains = ConvexChains.begin() + indexClusters_reactive;
	std::set<int>::const_iterator iter_convexchain;
	for( iter_convexchain = iter_ConvexChains->begin(); iter_convexchain != iter_ConvexChains->end(); iter_convexchain++ )
	{
		int i = *iter_convexchain;
		cv::Point2f pt;
		scan2CartesianInMeter(las, i, pt);
		obstaclePrevious.push_back(pt);
	}
	odometryPrevious.copyFrom(odometry);
}

// previous reactive obstacle tracking
void LaserSensing::reactiveObstacleTrack( const LaserScan * las,								// raw laser data
											 const std::vector<std::set <int> > &ConvexChains,	// clusters after merging
											 const Pose &odometry,								// odometry data
											 const std::vector<cv::Point2f> &obstaclePrevious,
											 const Pose &odometryPrevious,
											 const float threshold,								// tracking accuracy
											 int &indexClusters_reactive)						// which obstacle to react
{
	int num_match_max = 0;
	int num_match = 0;
	std::vector<std::set <int> >::const_iterator iter_ConvexChains;
	int index_track = 0;
	float dis;
	float dis_min = 2;
	for( iter_ConvexChains = ConvexChains.begin(); iter_ConvexChains != ConvexChains.end(); index_track++, iter_ConvexChains++ )
	{
		num_match = 0;
		std::set<int>::const_iterator iter_convexchain;
		for( iter_convexchain = iter_ConvexChains->begin(); iter_convexchain != iter_ConvexChains->end(); iter_convexchain++ )
		{
			cv::Point2f pt;
			scan2CartesianInMeter(las, *iter_convexchain, pt);
			// transform the pt into previous odometry coordinates
			// set the previous odometry as the relative coordinate
			Transform2D trans2d(odometryPrevious);			// set the previous odometry as the base frame
			// transform a current point to the base frame
			trans2d.transform_to_relative(pt);

			dis_min = 2;
			for ( int i = 0; i < obstaclePrevious.size(); i++ )
			{
				dis = distance2d( pt, obstaclePrevious[i] );
				if ( dis < dis_min )
					dis_min = dis;
			}
			if ( dis_min < threshold )
				num_match ++;


//			cv::Point2f pt;
//			scan2CartesianInMeter(las, *iter_convexchain, pt);
//			if( distance2d(pt,cv::Point2f(predictor._pose.p)) <= robot._radius + robot._disFollowing // define the close distance between the robot and ostacles
//					&& b_collision == false )
//			{
//				b_collision = true;
//				indexClusters_reactive = index_reactive;
//			}
		}
		if ( num_match > 1 )
			indexClusters_reactive = index_track;
	}
}

