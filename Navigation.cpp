#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Navigation.h"

Navigation::Navigation() {}

bool Navigation::initLaser(const float scaleLaser, const float angularResolution, const short maximumRadius)
{
	_scaleLaser = scaleLaser;
	_angularResolution = angularResolution;
	_maximumRadius = maximumRadius;
	return true;
}

bool Navigation::initLaser(const float scaleLaser, const float angularResolution, const short maximumRadius, const cv::Mat I)
{
	_scaleLaser = scaleLaser;
	_angularResolution = angularResolution;
	_maximumRadius = maximumRadius;
	_scale_w = (float)I.cols/(_maximumRadius*2);
	_scale_h = (float)I.rows/(_maximumRadius*2);
	return true;
}

bool Navigation::drawLaserCoordinate(const float width_area, const float height_area, cv::Mat &I)
{
	if( _maximumRadius == NULL )
			std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_maximumRadius*2);
	float scale_h = (float)I.rows/(_maximumRadius*2);
	cv::Point2i pt_c(I.cols/2,I.rows/2);

	cv::line(I, cv::Point(0, I.rows/2), cv::Point(I.cols,I.rows/2), cv::Scalar(255,255,255), 1, 8 ,0);
	cv::line(I, cv::Point(I.cols/2, 0), cv::Point(I.cols/2,I.rows), cv::Scalar(255,255,255), 1, 8 ,0);

	for ( int i = 1; i < 5; i++ )
	{
		cv::Size axes1( _maximumRadius*scale_w*i/5, _maximumRadius*scale_h*i/5 );
		cv::ellipse( I, pt_c, axes1, 0, 0, 360, cv::Scalar(255,255,255), 1, 8, 0);
	}

    cv::Size axes( _maximumRadius*scale_w, _maximumRadius*scale_h );
    double angle = 0;
    double startAngle = -210;
    double endAngle =  30;
    //cv::Scalar color(0,0,255);
    int thickness = 1;
    int lineType = 8;
    int shift = 0;
    cv::ellipse( I, pt_c, axes, angle, startAngle, endAngle, cv::Scalar(0,0,255), thickness, lineType, shift);
    cv::Point pt_l, pt_r;
    pt_l.x = _maximumRadius*(1-std::cos((float)PI/6))*scale_w;
    pt_l.y = _maximumRadius*(1+std::sin((float)PI/6))*scale_h;
    pt_r.x = _maximumRadius*(1+std::cos((float)PI/6))*scale_w;
    pt_r.y = pt_l.y;
    cv::line(I, pt_c, pt_l, cv::Scalar(0,0,255), 1, 8 ,0);
    cv::line(I, pt_c, pt_r, cv::Scalar(0,0,255), 1, 8 ,0);

	return true;
}

bool Navigation::drawLaser(const LaserScan *las, const float width_area, const float height_area, cv::Mat &I)
{
	if( _maximumRadius == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_maximumRadius*2);
	float scale_h = (float)I.rows/(_maximumRadius*2);
	for (int i = 0; i < las->size; i++)
	{
		if ( las->data[i] < _maximumRadius )
		{
			// las scan to cartesian coordiante
			cv::Point2f pt1;
			scan2Cartesian(las, i, pt1);
			//cartesian coordiante to draw window
			cv::Point2i pt2;
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 1, cv::Scalar(255,0,0), 1, 8, 1);
		}
	}
	return true;
}

bool Navigation::drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector<int> status, cv::Mat &I)
{
	if( _maximumRadius == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_maximumRadius*2);
	float scale_h = (float)I.rows/(_maximumRadius*2);
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

			cv::circle(I, pt2, 1, cv::Scalar(255,0,0), 1, 8, 1);
		}
	}
	return true;
}

bool Navigation::drawLaser(const LaserScan *las, const float width_area, const float height_area, const std::vector< std::vector<int> > Clusters, cv::Mat &I)
{
	if( _maximumRadius == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_maximumRadius*2);
	float scale_h = (float)I.rows/(_maximumRadius*2);

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
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);

			cv::circle(I, pt2, 1, scale, 1, 8, 1);

			if (iter_cluster == iter_Clusters->begin() || iter_cluster == iter_Clusters->end()-1)
				cv::circle(I, pt2, 6, scale, 1, 8, 1);
		}
	}
	return true;
}

bool Navigation::drawLaser(const LaserScan *las, const float width_area, const float height_area,
							const std::vector< std::vector<int> > Clusters, const std::vector<std::set<int> > PolygonalChains, cv::Mat &I)
{
	if( _maximumRadius == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_maximumRadius*2);
	float scale_h = (float)I.rows/(_maximumRadius*2);

	std::vector< std::set<int> >::const_iterator iter_PolygonalChains;
	std::vector< std::vector<int> >::const_iterator iter_Clusters = Clusters.begin();
	for ( iter_PolygonalChains = PolygonalChains.begin(); iter_PolygonalChains != PolygonalChains.end(); iter_PolygonalChains++, iter_Clusters++ )
	{
		//  iter_i points to an element of Clusters
		std::set<int>::const_iterator iter_polygonalchain, iter_up;
		cv::Scalar scale = cv::Scalar(rand()%255, rand()%255, rand()%255,0);
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
				cv::circle(I, pt2, 1, scale, 1, 8, 1);
			}

			// draw polygonal chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_polygonalchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 6, scale, 1, 8, 1);
			n++;
		}
	}
	return true;
}

// label the front obstacle with red
bool Navigation::drawObstacleReactive(const LaserScan *las, const float width_area, const float height_area, const std::vector<std::vector<int> > Clusters,
								const std::vector<std::set<int> > PolygonalChains, const bool b_avoid, const int index_avoid, cv::Mat &I)
{
	if( _maximumRadius == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_maximumRadius*2);
	float scale_h = (float)I.rows/(_maximumRadius*2);

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
					cv::circle(I, pt2, 2, cv::Scalar(0, 0, 255, 0), -1, 8, 1);
				else																// do not draw other obstacles
					cv::circle(I, pt2, 2, cv::Scalar(255, 0, 0, 0), -1, 8, 1);
			}

			// draw polygonal chains point
			// las scan to cartesian coordiante
			scan2Cartesian(las, *iter_polygonalchain, pt1);
			//cartesian coordiante to draw window
			cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
			cv::circle(I, pt2, 6, cv::Scalar(0, 0, 255, 0), 1, 8, 1);				// do not draw the segmentation
			n++;
		}
	}
	return true;
}

// draw the nearest point on the reactive obstacle
bool Navigation::drawNearestPoint(const LaserScan *las, const float width_area, const float height_area, const cv::Point2f ptObsNear, cv::Mat &I)
{
	if( _maximumRadius == NULL )
	std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_maximumRadius*2);
	float scale_h = (float)I.rows/(_maximumRadius*2);

	//cartesian coordiante to draw window
	cv::Point2i pt2;
	cartesian2DrawWindows(ptObsNear, scale_w, scale_h, I.cols, I.rows, pt2);
	cv::circle(I, pt2, 6, cv::Scalar(0,255,0), 1, 8, 1);	// label the nearest point
	return true;
}

// draw the B-spline of reactive obstacle boundary
bool Navigation::drawBSpline(const float width_area, const float height_area, const std::vector<Pose> curve_BSpline, cv::Mat &I)
{
	if( _maximumRadius == NULL )
		std::cout << "_maximumRadius is not initialized ...\n";
	float scale_w = (float)I.cols/(_maximumRadius*2);
	float scale_h = (float)I.rows/(_maximumRadius*2);

	std::vector< Pose >::const_iterator iter_CurveBSpline;
	for ( iter_CurveBSpline = curve_BSpline.begin(); iter_CurveBSpline != curve_BSpline.end(); iter_CurveBSpline++ )
	{
		cv::Point2f pt1;	// in cartesian coordiante in mm
		pt1.x = (*iter_CurveBSpline).p.x * 1000;
		pt1.y = (*iter_CurveBSpline).p.y * 1000;
		cv::Point2i pt2;	// in window coordinate
		//cartesian coordiante to draw window
		cartesian2DrawWindows(pt1, scale_w, scale_h, I.cols, I.rows, pt2);
		cv::circle(I, pt2, 2, cv::Scalar(0, 255, 0, 0), -1, 8, 1);
	}
	return true;
}

// draw the estimated visual feature
int Navigation::drawFeatureEstimation(const std::vector<cv::Point2f> &pts,
										const int state,
										cv::Mat &I_1,
										cv::Mat &I_track,
										cv::Mat &I)
{
	I = cv::Mat( cv::Size(640*3,480), CV_8UC3, cv::Scalar::all(255) );
	if ( state == 1 )	// target tracking
	{
		I_track.copyTo(I(cv::Rect(I_track.cols,0, I_track.cols, I_track.rows )));
//		for ( int i = 0; i < pts.size(); i++ )
//		{
//			cv::circle(I, cv::Point(I_track.cols + pts[i].x,pts[i].y), 8, cv::Scalar(255,0,0),-1,8,1);
//		}
	}
	else
	{
		I_1.copyTo(I(cv::Rect(I_track.cols,0, I_track.cols, I_track.rows )));

		cv::circle(I, cv::Point(I_1.cols + pts[0].x, pts[0].y), 8, cv::Scalar(255,255,0),-1,8,1);
		cv::circle(I, cv::Point(I_1.cols + pts[1].x, pts[1].y), 8, cv::Scalar(0,0,255),-1,8,1);
		cv::circle(I, cv::Point(I_1.cols + pts[2].x, pts[2].y), 8, cv::Scalar(255,128,0),-1,8,1);
		cv::circle(I, cv::Point(I_1.cols + pts[3].x, pts[3].y), 8, cv::Scalar(255,0,255),-1,8,1);

//		for ( int i = 0; i < pts.size(); i++ )
//		{
//			cv::circle(I, cv::Point(I_1.cols + pts[i].x, pts[i].y), 8, cv::Scalar(255,0,0),-1,8,1);
//		}
	}

	return 0;
}

// laser cartesian coordinate, same as robot coordinate (in millimeter)
//
//					^ x
//					|
//					|
//					|
//  y <-------------
bool Navigation::scan2Cartesian(const LaserScan *las, int index, cv::Point2f &pt)	// in millimeter
{
	float angle_trans = 2*PI/3;
    float theta = angle_trans - index * _angularResolution;
    theta = pi_to_pi(theta);
    pt.x = las->data[index] * std::cos(theta);
    pt.y = las->data[index] * std::sin(theta);
	return true;
}

void Navigation::scan2CartesianInMeter(const LaserScan *las, int index, cv::Point2f &pt)	// in meter
{
	float angle_trans = 2*PI/3;
	float theta = angle_trans - index * _angularResolution;
	theta = pi_to_pi(theta);
	pt.x = las->data[index] * std::cos(theta)/(float)(_scaleLaser);
	pt.y = las->data[index] * std::sin(theta)/(float)(_scaleLaser);
}

bool Navigation::cartesian2DrawWindows(const cv::Point2f pt1, const float scale_w, const float scale_h,
										const int width_win, const int height_win, cv::Point2i &pt2)
{
	pt2.x = (int) ( -pt1.y*scale_w + width_win/2 + 0.5 );
	pt2.y = (int) ( height_win/2 - pt1.x*scale_h + 0.5 );
	return true;
}

int Navigation::laserNoiseFilter(const LaserScan * las, const float threshold, std::vector<int> &status)
{
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

		if ( las->data[i] >= _maximumRadius )	// out of range is also isolated points
			s = 0;

		dis_im1 = dis_ip1;

		status.push_back(s);
	}
	return 0;
}

int Navigation::laserCluster(const LaserScan *las, const std::vector<int> status,
								const short threshold, const short length_minimum,
								std::vector<std::vector<int>* > &Clusters)
{
	short r_im1 = 0;
	short r_i = 0;
	std::vector<int> *cluster = NULL;
	for (int i = 0; i < las->size; i++)
	{
		r_i = las->data[i];
		if ( cluster != NULL )    // already start the current cluster, and finding the inside points of the cluster
		{
			if ( std::abs(r_i - r_im1) < threshold && status[i] == 1 )	//	is a inside point
			{
				cluster->push_back(i);
				if (i == las->size - 1)									// the last scan, stop the current cluster
				{
					Clusters.push_back(cluster);
					cluster = NULL;
				}
			}
			else														// the end of the current cluster
			{
				if (cluster->size() > length_minimum )					// is a real cluster, push back
				{
					Clusters.push_back(cluster);
				}
				else													// not a real cluster, release the memory
				{
					delete cluster;
				}
				cluster = NULL;											// end the current cluster
			}
		}

		if ( cluster == NULL			// wait to start a new cluster
			&& status[i] == 1 )			// not a noise
		{
			cluster = new std::vector<int>();		// start a current cluster
			cluster->push_back(i); 					// the left point of the cluster
		}

		r_im1 = r_i;
	}
	return 0;
}

// laser clustering/segmentation
// it divides the filtered points into groups of close points and discards groups consisting of too few points
int Navigation::laserCluster(const LaserScan *las, const std::vector<int> status,
								const short threshold, const short length_minimum,
								std::vector< std::vector<int> > &Clusters)
{
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

		if ( cluster.empty()						// wait to start a new cluster
			&& status[i] == 1 )						// not a noise
		{
			cluster.push_back(i); 					// the left point of the cluster
		}

		r_im1 = r_i;
	}
	return 0;
}

int Navigation::laserClusterMerge(const LaserScan *las, const float threshold, std::vector<std::vector<int>* > &Clusters)
{
//	if (Clusters.size() > 1)
//	{
//		std::vector<std::vector<int>* > it;
//		int i = 0;
//		for (it = Clusters.begin(); it < Clusters.end() - 1; it++)
//		{
//			int index_curr_cluster_end = it.size() -1;
//			int index_next_cluster_begin = (it+1)[0];
//			double dis = 0;
//			cv::Point2f pt_curr, pt_next;
//			scan2Cartesian(las, i, pt_curr);
//			scan2Cartesian(las, i, pt_next);
//			dis = distance2d(pt_curr, pt_next);				// here we use distance between two point instead of using r1 and r2
//															// because it is not sure that index_curr_cluster_last
//															// and index_next_cluster_first are two adjacent scan
//			if (dis < threshold)
//			{
//				foreach (int ii in Clusters[i + 1])
//				{
//					Clusters[i].Add(ii);
//				}
//				Clusters.Remove(Clusters[i + 1]);
//				i--;
//			}
//			i++;
//		}
//	}

//	if (Clusters.size() > 1)
//	{
//		for ( int i = 0; i < Clusters.size() - 1; i++)
//		{
//			int index_curr_cluster_end = Clusters[i]-> ;
//			int index_next_cluster_begin = (it+1)[0];
//			double dis = 0;
//			cv::Point2f pt_curr, pt_next;
//			scan2Cartesian(las, i, pt_curr);
//			scan2Cartesian(las, i, pt_next);
//			dis = distance2d(pt_curr, pt_next);				// here we use distance between two point instead of using r1 and r2
//															// because it is not sure that index_curr_cluster_last
//															// and index_next_cluster_first are two adjacent scan
//			if (dis < threshold)
//			{
//				foreach (int ii in Clusters[i + 1])
//				{
//					Clusters[i].Add(ii);
//				}
//				Clusters.Remove(Clusters[i + 1]);
//				i--;
//			}
//			i++;
//		}
//	}
	return 0;
}

// computer the the Cartesian distance of the adjacent end points between two adjacent clusters
int Navigation::laserClusterMerge(const LaserScan *las, const float threshold, std::vector< std::vector<int> > &Clusters)
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
	return 0;
}

int Navigation::deleteClusters(std::vector<std::vector<int>* > &Clusters)
{
	while (!Clusters.empty())
	{
		delete Clusters.back();
		Clusters.pop_back();
	}
	return 0;
}

int Navigation::LaserRecursionCalcPolygonalChain( const LaserScan *las,
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
	return 0;
}

// find an open polygonal chain to each cluster
int Navigation::laserLineFitting(const LaserScan *las,
									const float threshold,
									const std::vector<std::vector<int> > Clusters,
									std::vector<std::set <int> > &PolygonalChains)	 // polygonalchains restore the index of las->data
{
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

	return 0;
}

float Navigation::getScaleLaser()
{
	return _scaleLaser;
	}

