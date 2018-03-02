/*
 * myDraw.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: robot
 */

#include "myDraw.h"
#include "LaserSensing.h"
#include "myUtilities.h"
#include "highgui.h"

MyDraw::MyDraw()	{}

double MyDraw::angularResolutionGet()
{
	return _angular_resolution;
}

void MyDraw::angularResolutionSet(double angular_resolution)
{
	_angular_resolution = angular_resolution;
}

int MyDraw::maximumRadiusGet()
{
	return _maximum_radius;
}

void MyDraw::maximumRadiusSet(int maximum_radius)
{
	_maximum_radius = maximum_radius;
}

double MyDraw::rangeScaleGet()
{
	return _range_scale;
}

void MyDraw::rangeScaleSet(double range_scale)
{
	_range_scale = range_scale;
}

int MyDraw::scanAngleGet()
{
	return _scan_angle;
}

void MyDraw::scanAnglSet(int scan_angle)
{
	_scan_angle = scan_angle;
}

int MyDraw::stepNumberGet()
{
	return _step_number;
}

void MyDraw::stepNumberSet(int step_number)
{
	_step_number = step_number;
}

int MyDraw::radiusNaviGet()
{
	return _radius_navi;
}

void MyDraw::radiusNaviSet(int radius_navi)
{
	_radius_navi = radius_navi;
}

void MyDraw::drawLaserCoordinate(const float width_area, const float height_area, cv::Mat &I)
{
	if( _radius_navi == NULL )
			std::cout << "_radius_navi is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);
	cv::Point2i pt_c(I.cols/2,I.rows/2);

	cv::line(I, cv::Point(0, I.rows/2), cv::Point(I.cols,I.rows/2), cv::Scalar(255,255,255), 1, 8 ,0);
	cv::line(I, cv::Point(I.cols/2, 0), cv::Point(I.cols/2,I.rows), cv::Scalar(255,255,255), 1, 8 ,0);

	// show the ranges
//	for ( int i = 1; i < 5; i++ )
//	{
//		cv::Size axes1(  _radius_navi*scale_w*i/5,  _radius_navi*scale_h*i/5 );
//		// cv::ellipse( I, pt_c, axes1, 0, 0, 360, cv::Scalar(255,255,255), 1, 8, 0);
//		cv::ellipse( I, pt_c, axes1, 0, 0, 360, cv::Scalar(125,125,125), 1, 8, 0);
//	}

	// draw the range region
    cv::Size axes(  _radius_navi*scale_w,  _radius_navi*scale_h );
    double angle = 0;
    double startAngle = -210;
    double endAngle =  30;
    int thickness = 1;
    int lineType = 8;
    int shift = 0;
    cv::ellipse( I, pt_c, axes, angle, startAngle, endAngle, cv::Scalar(0,0,255), thickness, lineType, shift);
    cv::Point pt_l, pt_r;
    pt_l.x =  _radius_navi*(1-std::cos((float)PI/6))*scale_w;
    pt_l.y =  _radius_navi*(1+std::sin((float)PI/6))*scale_h;
    pt_r.x =  _radius_navi*(1+std::cos((float)PI/6))*scale_w;
    pt_r.y = pt_l.y;
    cv::line(I, pt_c, pt_l, cv::Scalar(0,0,255), 1, 8 ,0);
    cv::line(I, pt_c, pt_r, cv::Scalar(0,0,255), 1, 8 ,0);
}

// draw raw measurements of laser scane
void MyDraw::drawLaser(const LaserScan *las, const float width_area, const float height_area, cv::Mat &I)
{
	if( _radius_navi == NULL )
				std::cout << "_radius_navi is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);
	for (int i = 0; i < las->size; i++)
	{
		if ( las->data[i] < _maximum_radius )
		{
			// las scan to cartesian coordiante
			cv::Point2f pt1;
			scan2Cartesian(las, i, pt1);
			//cartesian coordiante to draw window
			cv::Point2i pt2;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 2, cv::Scalar(255,0,0), -1, 8, 0);
		}
	}
}

// draw noise filtered of laser scan
void MyDraw::drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<int> status, cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_radius_navi is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);
	for (int i = 0; i < las->size; i++)
	{
		if ( status[i] == 1 )
		{
			// las scan to cartesian coordiante
			cv::Point2f pt1;
			scan2Cartesian(las, i, pt1);
			//cartesian coordiante to draw window
			cv::Point2i pt2;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);

			cv::circle(I, pt2, 2, cv::Scalar(255,0,0), -1, 8, 0);
		}
	}
}

// draw cluster
void MyDraw::drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector< std::vector<int> > Clusters, cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_radius_navi is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::vector<int> >::const_iterator iter_Clusters;
	for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end(); iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::vector<int>::const_iterator iter_cluster;
		cv::Scalar scale = cv::Scalar(rand()%255, rand()%255, rand()%255,0);
		//cv::Scalar scale = cv::Scalar(255,0,0);
		for( iter_cluster = iter_Clusters->begin(); iter_cluster < iter_Clusters->end(); iter_cluster++ )
		{
			// las scan to cartesian coordiante
			cv::Point2f pt1;
			scan2Cartesian(las, *iter_cluster, pt1);
			//cartesian coordiante to draw window
			cv::Point2i pt2;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);

			cv::circle(I, pt2, 2, scale, -1, 8, 0);

//			if (iter_cluster == iter_Clusters->begin() || iter_cluster == iter_Clusters->end()-1)
//				cv::circle(I, pt2, 6, scale, 1, 8, 1);
		}
	}
}

// draw Polygonal Chains after line fitting
void MyDraw::drawLaser(const LaserScan *las, const float width_area, const float height_area,
							const std::vector< std::vector<int> > Clusters, const std::vector<std::set<int> > PolygonalChains, cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::set<int> >::const_iterator iter_PolygonalChains;
	std::vector< std::vector<int> >::const_iterator iter_Clusters = Clusters.begin();
	for ( iter_PolygonalChains = PolygonalChains.begin(); iter_PolygonalChains != PolygonalChains.end(); iter_PolygonalChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_polygonalchain, iter_up;
		cv::Scalar scale = cv::Scalar(rand()%255, rand()%255, rand()%255,0);
		for( iter_polygonalchain = iter_PolygonalChains->begin(); iter_polygonalchain != iter_PolygonalChains->end(); iter_polygonalchain++ )
		{
			cv::Point2f pt1;
			cv::Point2i pt2;

			// draw laser data
			std::vector<int>::const_iterator iter_cluster;
			for ( iter_cluster = iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++ )
			{
				// las scan to cartesian coordiante
				scan2Cartesian(las, *iter_cluster, pt1);
				//cartesian coordiante to draw window
				cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
				cv::circle(I, pt2, 2, scale, -1, 8, 0);
			}

			// draw polygonal chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_polygonalchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 6, scale, 1, 8, 0);
		}
	}
}

// draw only Polygonal Chains after line fitting
void MyDraw::drawPolygonalChains( const LaserScan *las,
								  const float width_area,
								  const float height_area,
								  const std::vector<std::vector<int> > Clusters,
								  const std::vector<std::set<int> > PolygonalChains,
								  cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::set<int> >::const_iterator iter_PolygonalChains;
	std::vector< std::vector<int> >::const_iterator iter_Clusters = Clusters.begin();
	for ( iter_PolygonalChains = PolygonalChains.begin(); iter_PolygonalChains != PolygonalChains.end(); iter_PolygonalChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_polygonalchain, iter_up;
		// cv::Scalar scale = cv::Scalar(rand()%255, rand()%255, rand()%255,0);
		cv::Scalar scale;
		for( iter_polygonalchain = iter_PolygonalChains->begin(); iter_polygonalchain != iter_PolygonalChains->end(); iter_polygonalchain++ )
		{
			cv::Point2f pt1;
			cv::Point2i pt2;

			// draw laser data
			std::vector<int>::const_iterator iter_cluster;
			for ( iter_cluster = iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++ )
			{
				// las scan to cartesian coordiante
				scan2Cartesian(las, *iter_cluster, pt1);
				//cartesian coordiante to draw window
				cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
				scale = cv::Scalar(255,0,0);
				cv::circle(I, pt2, 2, scale, -1, 8, 0);
			}

			// draw polygonal chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_polygonalchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			scale = cv::Scalar::all(0);
			cv::circle(I, pt2, 6, scale, 1, 8, 0);
		}
	}
}

// draw only Convex Chains after line fitting
void MyDraw::drawConvexChains( const LaserScan *las,
							   const float width_area,
							   const float height_area,
							   const std::vector<std::vector<int> > Clusters,
							   const std::vector<std::set<int> > ConvexChains,
							   cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::set<int> >::const_iterator iter_ConvexChains;
	std::vector< std::vector<int> >::const_iterator iter_Clusters = Clusters.begin();
	for ( iter_ConvexChains = ConvexChains.begin(); iter_ConvexChains != ConvexChains.end(); iter_ConvexChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_convexchain, iter_up;
		// cv::Scalar scale = cv::Scalar(rand()%255, rand()%255, rand()%255,0);
		cv::Scalar scale;
		for( iter_convexchain = iter_ConvexChains->begin(); iter_convexchain != iter_ConvexChains->end(); iter_convexchain++ )
		{
			cv::Point2f pt1;
			cv::Point2i pt2;

			// draw laser data
			std::vector<int>::const_iterator iter_cluster;
			for ( iter_cluster = iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++ )
			{
				// las scan to cartesian coordiante
				scan2Cartesian(las, *iter_cluster, pt1);
				//cartesian coordiante to draw window
				cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
				scale = cv::Scalar(255,0,0);
				cv::circle(I, pt2, 2, scale, -1, 8, 0);
			}

			// draw convex chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_convexchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			scale = cv::Scalar(0,0,255);
			cv::circle(I, pt2, 6, scale, 1, 8, 0);
		}
	}
}

/**
 *  draw with given color
 */
// draw cluster
void MyDraw::drawLaser( const LaserScan *las,
						const float width_area,
						const float height_area,
						const std::vector< std::vector<int> > Clusters,
						const cv::Scalar &color,
						cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_radius_navi is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::vector<int> >::const_iterator iter_Clusters;
	for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end(); iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::vector<int>::const_iterator iter_cluster;
		for( iter_cluster = iter_Clusters->begin(); iter_cluster < iter_Clusters->end(); iter_cluster++ )
		{
			// las scan to cartesian coordiante
			cv::Point2f pt1;
			scan2Cartesian(las, *iter_cluster, pt1);
			//cartesian coordiante to draw window
			cv::Point2i pt2;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);

			cv::circle(I, pt2, 2, color, -1, 8, 0);

//			if (iter_cluster == iter_Clusters->begin() || iter_cluster == iter_Clusters->end()-1)
//				cv::circle(I, pt2, 6, scale, 1, 8, 1);
		}
	}
}

// draw only Polygonal Chains after line fitting
void MyDraw::drawPolygonalChains( const LaserScan *las,
								  const float width_area,
								  const float height_area,
								  const std::vector<std::vector<int> > Clusters,
								  const std::vector<std::set<int> > PolygonalChains,
								  const cv::Scalar &color,
								  cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::set<int> >::const_iterator iter_PolygonalChains;
	std::vector< std::vector<int> >::const_iterator iter_Clusters = Clusters.begin();
	for ( iter_PolygonalChains = PolygonalChains.begin(); iter_PolygonalChains != PolygonalChains.end(); iter_PolygonalChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_polygonalchain, iter_up;
		for( iter_polygonalchain = iter_PolygonalChains->begin(); iter_polygonalchain != iter_PolygonalChains->end(); iter_polygonalchain++ )
		{
			cv::Point2f pt1;
			cv::Point2i pt2;

			// draw laser data
			std::vector<int>::const_iterator iter_cluster;
			for ( iter_cluster = iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++ )
			{
				// las scan to cartesian coordiante
				scan2Cartesian(las, *iter_cluster, pt1);
				//cartesian coordiante to draw window
				cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
				cv::circle(I, pt2, 2, color, -1, 8, 0);
			}

			// draw polygonal chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_polygonalchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 6, cv::Scalar::all(0), 1, 8, 0);
		}
	}
}

// draw only Convex Chains after line fitting
void MyDraw::drawConvexChains( const LaserScan *las,
							   const float width_area,
							   const float height_area,
							   const std::vector<std::vector<int> > Clusters,
							   const std::vector<std::set<int> > ConvexChains,
							   const cv::Scalar &color,
							   cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::set<int> >::const_iterator iter_ConvexChains;
	std::vector< std::vector<int> >::const_iterator iter_Clusters = Clusters.begin();
	for ( iter_ConvexChains = ConvexChains.begin(); iter_ConvexChains != ConvexChains.end(); iter_ConvexChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_convexchain, iter_up;
		for( iter_convexchain = iter_ConvexChains->begin(); iter_convexchain != iter_ConvexChains->end(); iter_convexchain++ )
		{
			cv::Point2f pt1;
			cv::Point2i pt2;

			// draw laser data
			std::vector<int>::const_iterator iter_cluster;
			for ( iter_cluster = iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++ )
			{
				// las scan to cartesian coordiante
				scan2Cartesian(las, *iter_cluster, pt1);
				//cartesian coordiante to draw window
				cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
				cv::circle(I, pt2, 2, color, -1, 8, 0);
			}

			// draw convex chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_convexchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 6, cv::Scalar(0,0,255), 1, 8, 0);
		}
	}
}

/**
 *  draw results with random colors
 *  clustering and merge
 *  polygonal chains
 *  convex chains
 */
//// draw with random colors
void MyDraw::drawLaserResults( const LaserScan *las,
								const float width_area,
								const float height_area,
								const std::vector<std::vector<int> > Segments,
								const std::vector<std::vector<int> > Clusters,
								const std::vector<std::set<int> > PolygonalChains,
								const std::vector<std::set<int> > ConvexChains,
								cv::Mat &I_laserSegments,
								cv::Mat &I_laserClusteringAndMerging,
								cv::Mat &I_laserPolygonalChains,
								cv::Mat &I_laserConvexChains)
{
	if( _radius_navi == NULL )
		std::cout << "_radius_navi is not initialized ...\n";
	float scale_w = (float)I_laserClusteringAndMerging.cols/(_radius_navi*2);
	float scale_h = (float)I_laserClusteringAndMerging.rows/(_radius_navi*2);

	// draw segments
	std::vector< std::vector<int> >::const_iterator iter_Segments;
	for ( iter_Segments = Segments.begin(); iter_Segments != Segments.end(); iter_Segments++ )
	{
		//  iter_i points to an element of Clusters
		std::vector<int>::const_iterator iter_segment;
		cv::Scalar scale = cv::Scalar(rand()%255, rand()%255, rand()%255,0);
		for( iter_segment = iter_Segments->begin(); iter_segment < iter_Segments->end(); iter_segment++ )
		{
			// las scan to cartesian coordiante
			cv::Point2f pt1;
			scan2Cartesian(las, *iter_segment, pt1);
			//cartesian coordiante to draw window
			cv::Point2i pt2;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I_laserSegments.cols, I_laserSegments.rows, pt2);

			cv::circle(I_laserSegments, pt2, 2, scale, -1, 8, 0);
		}
	}


	std::vector< std::vector<int> >::const_iterator iter_Clusters;
	for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end(); iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::vector<int>::const_iterator iter_cluster;
		cv::Scalar scale = cv::Scalar(rand()%255, rand()%255, rand()%255,0);
		for( iter_cluster = iter_Clusters->begin(); iter_cluster < iter_Clusters->end(); iter_cluster++ )
		{
			// las scan to cartesian coordiante
			cv::Point2f pt1;
			scan2Cartesian(las, *iter_cluster, pt1);
			//cartesian coordiante to draw window
			cv::Point2i pt2;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I_laserClusteringAndMerging.cols, I_laserClusteringAndMerging.rows, pt2);

			cv::circle(I_laserClusteringAndMerging, pt2, 2, scale, -1, 8, 0);
		}
	}

	// draw polygonal chains
	I_laserClusteringAndMerging.copyTo(I_laserPolygonalChains);
	std::vector< std::set<int> >::const_iterator iter_PolygonalChains;
	for ( iter_PolygonalChains = PolygonalChains.begin(); iter_PolygonalChains != PolygonalChains.end(); iter_PolygonalChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_polygonalchain, iter_up;
		for( iter_polygonalchain = iter_PolygonalChains->begin(); iter_polygonalchain != iter_PolygonalChains->end(); iter_polygonalchain++ )
		{
			cv::Point2f pt1;
			cv::Point2i pt2;
			// draw polygonal chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_polygonalchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I_laserPolygonalChains.cols, I_laserPolygonalChains.rows, pt2);
			cv::circle(I_laserPolygonalChains, pt2, 6, cv::Scalar::all(0), 1, 8, 0);
		}
	}


	// draw convex chains
	I_laserClusteringAndMerging.copyTo(I_laserConvexChains);
	std::vector< std::set<int> >::const_iterator iter_ConvexChains;
	for ( iter_ConvexChains = ConvexChains.begin(); iter_ConvexChains != ConvexChains.end(); iter_ConvexChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_convexchain, iter_up;
		for( iter_convexchain = iter_ConvexChains->begin(); iter_convexchain != iter_ConvexChains->end(); iter_convexchain++ )
		{
			cv::Point2f pt1;
			cv::Point2i pt2;
			// draw convex chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_convexchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I_laserConvexChains.cols, I_laserConvexChains.rows, pt2);
			cv::circle(I_laserConvexChains, pt2, 6, cv::Scalar(0,0,255), 1, 8, 0);
		}
	}
}


/**
 *  draw results based on opencv function
 */
// draw contour curves
void MyDraw::drawContourCurves( const LaserScan *las,
								const float width_area,
								const float height_area,
								const std::vector< std::vector<cv::Point2f> > &contourCurves,
								cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::vector<cv::Point2f> >::const_iterator iter_contourCurves;
	for( iter_contourCurves = contourCurves.begin(); iter_contourCurves != contourCurves.end(); iter_contourCurves++ )
	{
		std::vector< cv::Point2f >::const_iterator iter_contourCurve;
		for ( iter_contourCurve = iter_contourCurves->begin(); iter_contourCurve != iter_contourCurves->end(); iter_contourCurve++ )
		{
			//cartesian coordiante to draw window
			cv::Point2f pt1;
			cv::Point2i pt2;
			pt1.x = iter_contourCurve->x;
			pt1.y = iter_contourCurve->y;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 2, cv::Scalar(255,0,0), -1, 8, 0);
		}
	}
}

// draw polygonal curves
void MyDraw::drawPolygonalCurves( const LaserScan *las,
								  const float width_area,
								  const float height_area,
								  const std::vector< std::vector<cv::Point2f> > &polygonalCurves,
								  cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::vector<cv::Point2f> >::const_iterator iter_polygonalCurves;
	for( iter_polygonalCurves = polygonalCurves.begin(); iter_polygonalCurves != polygonalCurves.end(); iter_polygonalCurves++ )
	{
		std::vector< cv::Point2f >::const_iterator iter_polygonalCurve;
		for ( iter_polygonalCurve = iter_polygonalCurves->begin(); iter_polygonalCurve != iter_polygonalCurves->end(); iter_polygonalCurve++ )
		{
			//cartesian coordiante to draw window
			cv::Point2f pt1;
			cv::Point2i pt2;
			pt1.x = iter_polygonalCurve->x;
			pt1.y = iter_polygonalCurve->y;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 6, cv::Scalar::all(125), 1, 8, 0);
		}
	}
}

// draw convex curves
void MyDraw::drawConvexCurves( const LaserScan *las,
								  const float width_area,
								  const float height_area,
								  const std::vector< std::vector<cv::Point2f> > &convexCurves,
								  cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::vector<cv::Point2f> >::const_iterator iter_convexCurves;
	for( iter_convexCurves = convexCurves.begin(); iter_convexCurves != convexCurves.end(); iter_convexCurves++ )
	{
		std::vector< cv::Point2f >::const_iterator iter_convexCurve;
		for ( iter_convexCurve = iter_convexCurves->begin(); iter_convexCurve != iter_convexCurves->end(); iter_convexCurve++ )
		{
			//cartesian coordiante to draw window
			cv::Point2f pt1;
			cv::Point2i pt2;
			pt1.x = iter_convexCurve->x;
			pt1.y = iter_convexCurve->y;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 6, cv::Scalar(0,0,255), 1, 8, 0);
		}

	}
}

/**
 *  visual object
 */
void MyDraw::drawTargetFourCorners(const std::vector<cv::Point2f> obj_corners, vpImage<vpRGBa> &I)
{
	if (obj_corners.size() == 4)
	{

	}
//	cv::line(img, cv::Point(obj_corners[0].x,obj_corners[0].y), cv::Point(obj_corners[1].x,obj_corners[1].y), cv::Scalar(0,255,0), 1, 8, 0 );
//	cv::line(img, cv::Point(obj_corners[1].x,obj_corners[1].y), cv::Point(obj_corners[2].x,obj_corners[2].y), cv::Scalar(0,255,0), 1, 8, 0 );
//	cv::line(img, cv::Point(obj_corners[2].x,obj_corners[2].y), cv::Point(obj_corners[3].x,obj_corners[3].y), cv::Scalar(0,255,0), 1, 8, 0 );
//	cv::line(img, cv::Point(obj_corners[3].x,obj_corners[3].y), cv::Point(obj_corners[0].x,obj_corners[0].y), cv::Scalar(0,255,0), 1, 8, 0 );
//	int radius = 10;
//	plot_dot_mark( img, cv::Point(obj_corners[0].x,obj_corners[0].y), radius,  cv::Scalar(255,255,0), 1, 8, 0 );
//	plot_dot_mark( img, cv::Point(obj_corners[1].x,obj_corners[1].y), radius,  cv::Scalar(0,0,255), 1, 8, 0 );
//	plot_dot_mark( img, cv::Point(obj_corners[2].x,obj_corners[2].y), radius,  cv::Scalar(255,128,0), 1, 8, 0 );
//	plot_dot_mark( img, cv::Point(obj_corners[3].x,obj_corners[3].y), radius,  cv::Scalar(255,0,255), 1, 8, 0 );
//
//	/* initialize font and add text */
//	PutLetterSubscript( img, "P", "0", cv::Point(obj_corners[0].x - 35, obj_corners[0].y - 5), cv::Scalar(255,255,0),  0.8, 0.8 );
//	PutLetterSubscript( img, "P", "1", cv::Point(obj_corners[1].x + 10, obj_corners[1].y -5 ), cv::Scalar(0,0,255),  0.8, 0.8 );
//	PutLetterSubscript( img, "P", "2", cv::Point(obj_corners[2].x + 10, obj_corners[2].y + 20), cv::Scalar(255,128,0),  0.8, 0.8 );
//	PutLetterSubscript( img, "P", "3", cv::Point(obj_corners[3].x - 35, obj_corners[3].y + 20), cv::Scalar(255,0,255),  0.8, 0.8 );
}

void MyDraw::drawFeatureEstimation(	const std::vector<cv::Point2f> &pts,
									const cv::Point2f g,
									cv::Mat &I)
{
	cv::circle(I, cv::Point( pts[0].x, pts[0].y), 8, cv::Scalar(255,255,0),-1,8,0);
	cv::circle(I, cv::Point( pts[1].x, pts[1].y), 8, cv::Scalar(0,0,255),-1,8,0);
	cv::circle(I, cv::Point( pts[2].x, pts[2].y), 8, cv::Scalar(255,128,0),-1,8,0);
	cv::circle(I, cv::Point( pts[3].x, pts[3].y), 8, cv::Scalar(255,0,255),-1,8,0);

	cv::line(I, cv::Point2i(I.cols/2, 0), cv::Point2i(I.cols/2, I.rows), cv::Scalar(0,0,255),1,8,0);
	cv::circle(I, g, 4, cv::Scalar(0,0,255),-1,8,0);
}

// draw target direction in the robot frame
// image												robot
//
//	----------------> x									 ^ x
//	|									 				 |
//	|									 				 |
//	|									   				 |
// y 									y <---------------
void MyDraw::drawTargetDirection( const float theta,	// direction
								  const float r,		// range of the direction line
								  const float scale_w,
								  const float scale_h,
								  cv::Mat &I)
{
	// in the robot frame
	cv::Point2f pt1_r(0,0);
	cv::Point2f pt2_r;
	pt2_r.x = r*std::cos(theta);
	pt2_r.y = r*std::sin(theta);

	// in the draw window
	cv::Point2i pt1, pt2;
	cartesian2DrawWindows(pt1_r, scale_w, scale_h, I.cols, I.rows, pt1 );
	cartesian2DrawWindows(pt2_r, scale_w, scale_h, I.cols, I.rows, pt2 );

	cv::line( I, pt1, pt2, cv::Scalar(0,0,255), 1, 8, 0 );
}

// laser cartesian coordinate, same as robot coordinate (in millimeter)
//
//					^ x
//					|
//					|
//					|
//  y <-------------
void MyDraw::scan2Cartesian(const LaserScan *las,
							int index,
							cv::Point2f &pt)	// in millimeter
{
	double angle_trans = 2*PI/3;
    double theta = angle_trans - index * _angular_resolution;
    theta = pi_to_pi(theta);
    pt.x = las->data[index] * std::cos(theta);
    pt.y = las->data[index] * std::sin(theta);
}

// transfrom laser scan in cartsian coordinate to image plane
//	-----------------> m (x)
//	|
//	|
//	|
//  v (y)
void MyDraw::cartesian2DrawWindows(	const cv::Point2f pt1,			// in millimeter
									const float scale_w,
									const float scale_h,
									const int width_win,
									const int height_win, cv::Point2i &pt2)
{
	pt2.x = (int) ( -pt1.y*scale_w + width_win/2 + 0.5 );
	pt2.y = (int) ( height_win/2 - pt1.x*scale_h + 0.5 );
}

// label the front obstacle with red
void MyDraw::drawObstacleReactive(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters,
								const std::vector<std::set<int> > PolygonalChains, const bool b_avoid, const int index_avoid, cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< std::set<int> >::const_iterator iter_PolygonalChains;
	std::vector< std::vector<int> >::const_iterator iter_Clusters = Clusters.begin();
	for ( iter_PolygonalChains = PolygonalChains.begin(); iter_PolygonalChains != PolygonalChains.end(); iter_PolygonalChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_polygonalchain, iter_up;
		int n = 0;

		for( iter_polygonalchain = iter_PolygonalChains->begin(); iter_polygonalchain != iter_PolygonalChains->end(); iter_polygonalchain++ )
		{
			cv::Point2f pt1;
			cv::Point2i pt2;

			// draw laser data
			std::vector<int>::const_iterator iter_cluster;
			for ( iter_cluster = iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++ )
			{
				// las scan to cartesian coordiante
				scan2Cartesian(las, *iter_cluster, pt1);
				//cartesian coordiante to draw window
				cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
				if( b_avoid == true && iter_PolygonalChains ==  PolygonalChains.begin() + index_avoid )
					cv::circle(I, pt2, 2, cv::Scalar(0, 0, 255, 0), -1, 8, 0);
				else																// do not draw other obstacles
					cv::circle(I, pt2, 2, cv::Scalar(255, 0, 0, 0), -1, 8, 0);
			}

			// draw polygonal chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_polygonalchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 6, cv::Scalar(0, 0, 255, 0), 1, 8, 0);				// do not draw the segmentation
			n++;
		}
	}
}

// draw the B-spline of reactive obstacle boundary
void MyDraw::drawBSpline(const float width_area, const float height_area, const std::vector<Pose> curve_BSpline, cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	std::vector< Pose >::const_iterator iter_CurveBSpline;
	for ( iter_CurveBSpline = curve_BSpline.begin(); iter_CurveBSpline != curve_BSpline.end(); iter_CurveBSpline++ )
	{
		cv::Point2f pt1;	// in cartesian coordiante in mm
		pt1.x = (*iter_CurveBSpline).p.x * 1000;
		pt1.y = (*iter_CurveBSpline).p.y * 1000;
		cv::Point2i pt2;	// in window coordinate
		//cartesian coordiante to draw window
		cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
		cv::circle(I, pt2, 2, cv::Scalar(0, 255, 0, 0), -1, 8, 0);
	}

}

// draw the nearest point on the reactive obstacle
void MyDraw::drawNearestPoint(const LaserScan *las, const float width_area, const float height_area, const cv::Point2f ptObsNear, cv::Mat &I)
{
	if( _radius_navi == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_radius_navi*2);
	float scale_h = (float)I.rows/(_radius_navi*2);

	//cartesian coordiante to draw window
	cv::Point2i pt2;
	cartesian2DrawWindows(ptObsNear, scale_w, scale_h, I.cols, I.rows, pt2);
	cv::circle(I, pt2, 6, cv::Scalar(0,255,0), 1, 8, 0);	// label the nearest point
}

// draw predicted trajecotry
int MyDraw::drawPredictedTrajecotry( const std::vector<cv::Point2f> &trajectoryPrediction,	// predicted trajectory in meter
									 const float scale_w,									// scale from the trajectory coordiantes to draw window
									 const float scale_h,
									 const float robot_R,									// radius of the robot
									 const bool b_avoid,
									 cv::Mat &I)
{
	cv::Scalar scale;
	for( int i = 0; i < trajectoryPrediction.size(); i++)
	{
		if ( b_avoid == true )
			scale =cv::Scalar(0,0,255);
		else
			scale =cv::Scalar(255,0,0);
		// transform meter coordinates into image plane
		//cartesian coordiante to draw window
		cv::Point2i pt2;
		cartesian2DrawWindows( trajectoryPrediction[i], scale_w, scale_h, I.cols, I.rows, pt2 );
		cv::circle(I, pt2, 0.5, scale, 1, 8, 0);
		if ( i == trajectoryPrediction.size()-1 )
			cv::circle(I, pt2, (int)(robot_R*scale_w+0.5), scale, 1, 8, 0);
	}
	//cv::polylines(I, trajectory, false, cv::Scalar{255,0,0},1,8,0);

	return 0;
}
