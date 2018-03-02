/*
 * motionControl.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: robot
 */

#include <ctime>
#include <iostream>
#include "motionControl.h"
#include "Navigation.h"
#include "myDraw.h"

std::string MotionState::get_State(const int n)
{
	std::string state;
	if( n == TARGET_DETECTION )
		state = "target detection";
	if(  n == TARGET_TRACKING	)
		state = "target tracking";
	if( n == TARGET_FINDING )
		state = "target finding";
	if( n == TARGET_FINDING_RANDOM )
			state = "target finding random";
	if( n == OBSTACLE_AVOIDANCE )
		state = "obstacle avoidance";
	if( n == STOP )
		state = "stop";
	if( n == END )
		state = "end";

	return state;
}

std::string PredictionMethod::get_State(const int n)
{
	std::string state;
	if( n == STRAIGHT_LINE_FOLLOWING )
		state = "straight line following";
	if(  n == CONSTANT_CONTROL	)
		state = "constant control";

	return state;
}

void motionControl::linearVelocity(const unsigned time_start,			// in milliseconds
									const unsigned time_stop,			// in milliseconds
									const unsigned time_current, 		// in milliseconds*
									const bool	   b_start,
									const bool	   b_stop,
									const float    Vl_accumulate,		// accumulate
									const float    VlMax,
									float          &newVl)				// per second
{
	float time_elapsed;

	if( b_start == true )
	{
		time_elapsed = (double)(time_current - time_start)/1000000.0F;
		newVl = (float)time_elapsed*Vl_accumulate;
	}
	else
		newVl = VlMax;

	if(  b_stop == true )
	{
		time_elapsed = (double)(time_current - time_stop)/1000000.0F;
		newVl = VlMax - 2*(float)time_elapsed*Vl_accumulate;
	}
}

// linear profile (constant acceleration)
void motionControl::linearVelocity(const std::string 	motion_mode,
									const unsigned		time_0,				// in milliseconds
									const unsigned 		time_current, 		// in milliseconds
									const float			Vl0,
									const float			Vlexpected,
									const float    		VlMax,
									const float    		Vl_acceleration,	// accumulate
									float          		&newVl)			// per second, also the current velocity
{
	float time_elapsed = (double)(time_current - time_0)/1000000.0F;
	float Sign = newVl < Vlexpected ? 1 : -1;

	if( motion_mode.compare("start") == 0 ||  motion_mode.compare("cruise") == 0 )
	{
		newVl = Vl0 + Sign*(float)time_elapsed*Vl_acceleration < Vlexpected ?  Vl0 + Sign*(float)time_elapsed*Vl_acceleration : Vlexpected;
	}

	if( motion_mode.compare("obstacleAvoidance") == 0 )		// exponential stabilization
	{
		newVl = (Vl0 + Sign*(float)time_elapsed*1.5*Vl_acceleration) > Vlexpected ? (Vl0 + Sign*(float)time_elapsed*1.5*Vl_acceleration): Vlexpected;
	}

	if(  motion_mode.compare("stop") == 0 )			// acceleration 2 times than normal case
	{
		newVl = (Vl0 + Sign*2*(float)time_elapsed*Vl_acceleration) > 0 ? (Vl0 + Sign*2*(float)time_elapsed*Vl_acceleration): 0;
	}

	if ( newVl > VlMax ) newVl = VlMax;
}


// based on predefined linear velocity profile
void motionControl::linearVelocityBasedOnPotentialCollisionDistance(	const bool b_collision,
																		const Robot &robot,			// constraints of the robot
																		const double s_path,		// potential collision distance
																		double &vl_expected,		// expected linear velocity
																		bool &b_escape)
{
	if( b_collision == true )
	{
		b_escape = false;
		if ( s_path < 0.2 )
			vl_expected = 0;
		if ( s_path > 0.2 && s_path <= 0.5 )
		{
			double k = (0.15 - 0)/(0.5 - 0.2);
			vl_expected = 0 + k*( s_path - 0.2 );
		}
		if ( s_path > 0.5 && s_path <=4 )
		{
			double k = (robot._vMax - 0.15)/(4 - 0.5);
			vl_expected = 0.15 + k*(s_path - 0.5);
		}
		if ( s_path > 4 )
			vl_expected = robot._vMax;
	}
	else
		vl_expected = robot._vMax;

	if ( vl_expected > robot._vMax )
		vl_expected = robot._vMax;
}

// odometry								robot
//
//					^ x									 ^ x
//					|									 |
//					|									 |
//					|									 |
// y <---------------					y <---------------
void motionControl::straightlineFollowing(const Pose odometry, double &Vl, double &Va)
{
	double delta_1, alpha_1, chi_1, k_1;
	delta_1 = odometry.p.y - 0;
	alpha_1 = odometry.phi - 0;
	alpha_1 = pi_to_pi(alpha_1);
	chi_1 = 0;
	k_1 = 1;
	Va = -Vl * (k_1 * delta_1 + alpha_1 + 2 * k_1 * std::sin(alpha_1) + chi_1 * std::cos(alpha_1));

//	double delta_1, alpha_1, chi_1, k_1;
//	delta_1 = 0 - odometry.p.y;
//	alpha_1 = 0 - odometry.phi;
//	alpha_1 = pi_to_pi(alpha_1);
//	chi_1 = 0;
//	k_1 = 1;
//	Va = -Vl * (k_1 * delta_1 + alpha_1 + 2 * k_1 * std::sin(alpha_1) - chi_1 * std::cos(alpha_1));
}

void motionControl::obstacleAvoidenceOK( const LaserScan *las,
											const std::vector<std::vector<int> >  &Clusters,
											const float thresholdX_avoid,
											const float thresholdY_avoid,
											int &indexClusters_avoid,
											bool &b_avoid)
{
	Navigation navi;
	navi.initLaser(1000, 0.006132813, 5000);
    std::vector< std::vector<int> >::const_iterator iter_Clusters;
    indexClusters_avoid = 0;
    int index_laserscan;
    int index_laserscan_temp = 0;
    int n = 0;
	for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end() - 1; n++, iter_Clusters++ )
	{
		std::vector<int>::const_iterator iter_cluster;
		for ( iter_cluster =  iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++ )
		{
			double dis = 0;
			cv::Point2f pt_near;
			navi.scan2Cartesian(las, *iter_cluster, pt_near);
			if ( 0 < pt_near.x && pt_near.x < thresholdX_avoid && std::abs(pt_near.y) < thresholdY_avoid )
			{
				b_avoid = true;
				index_laserscan_temp = ( *iter_Clusters->begin() + *(iter_Clusters->end()-1) )/2;
				if ( std::abs(index_laserscan_temp-las->size/2) < std::abs(index_laserscan-las->size/2) )	// select the front obstacle
					indexClusters_avoid = n;	index_laserscan = index_laserscan_temp;
				break;
			}
		}
	}
}

// select the reactive obstacle considering the primary task
// suppose the current primary task is straight line following
void motionControl::obstacleAvoidenceOK( const LaserScan *las,
							const std::vector<std::vector<int> >  &Clusters,
							const Pose odometry,
							const float thresholdX_avoid,
							const float thresholdY_avoid,
							int &indexClusters_avoid,
							bool &b_avoid)
{
	Navigation navi;
	navi.initLaser(1000, 0.006132813, 5000);
	std::vector< std::vector<int> >::const_iterator iter_Clusters;
	indexClusters_avoid = 0;
	int index_laserscan;
	int index_laserscan_temp = 0;
	int n = 0;
	for ( iter_Clusters = Clusters.begin(); iter_Clusters != Clusters.end(); n++, iter_Clusters++ )	// or iter_Clusters < Clusters.end()
	{
		std::vector<int>::const_iterator iter_cluster;
		for ( iter_cluster =  iter_Clusters->begin(); iter_cluster != iter_Clusters->end(); iter_cluster++ )
		{
			double dis = 0;
			cv::Point2f pt_near;
			navi.scan2Cartesian(las, *iter_cluster, pt_near);
			// transform pt_near to the world frame
			cv::Point2f pt_near_world;
			Transform2D transform2D_pt(odometry);
			pt_near_world.x = pt_near.x;
			pt_near_world.y = pt_near.y;
			transform2D_pt.transform_to_global(pt_near_world);

			if ( pt_near.x < thresholdX_avoid
					&& std::abs(pt_near.y) < thresholdY_avoid
					&& std::abs(pt_near_world.y)< 500 )			// for the obstacle on the target line
			{
				b_avoid = true;
				index_laserscan_temp = ( *iter_Clusters->begin() + *(iter_Clusters->end()-1) )/2;
				if ( std::abs(index_laserscan_temp-las->size/2) < std::abs(index_laserscan-las->size/2) )	// select the front obstacle
					indexClusters_avoid = n;	index_laserscan = index_laserscan_temp;
				break;
			}
		}
	}
}

// collision prediction for straight line following as the goal task
// straight line following task is exacuted in the world frame
// collision prediction is exacuted in the local frame (robot frame)
// output the reactive obstacle and the protential collision distance
void motionControl::collisioinPredictionStraightLineFollowing(	const LaserScan *las,
																Navigation &navi,
																const std::vector<std::vector<int> >  &Clusters,
																const Pose &odometry,		// odometry in the world frame
																const Robot &robot,
																const float newVl,
																const float newVa,
																const double preavoidDistance,
																bool &b_collision,
																int &indexClusters_reactive,	// decide to reactive with which obstacle
																double &s_path)				// protential collision distance)
{
	// initial
	s_path = 0;
	b_collision = false;
	double dt = 0.1;	// 0.1 second

	/*
	 * 	 inital the predictor
	 */
	Robot predictor(robot);
	predictor._pose.p.x = 0;	// robot in the original in the local coordination
	predictor._pose.p.y = 0;
	predictor._pose.phi = 0;
	if ( newVl < 0.2 )
		predictor._v = 0.2;
	else
		predictor._v = newVl;
	predictor._w = newVa;

	/*
	 * 	loop for motion prediction the same as the task straight line following
	 */
	bool b_break = false;
	while( s_path < preavoidDistance )		// predict 4 meter
	{
		/*
		 *  collision verification
		 *  output the index of reactive obstacle/clusters
		 */
		std::vector< std::vector<int> >::const_iterator iter_Clusters;
		int index_reactive = 0;
		for( iter_Clusters = Clusters.begin(); iter_Clusters < Clusters.end(); index_reactive++, iter_Clusters++ )
		{
			std::vector<int>::const_iterator iter_cluster;
			for( iter_cluster = iter_Clusters->begin(); iter_cluster < iter_Clusters->end(); iter_cluster++ )
			{
				cv::Point2f pt;
				navi.scan2CartesianInMeter(las,*iter_cluster,pt);
				if( distance2d(pt,cv::Point2f(predictor._pose.p)) < robot._radius + robot._disFollowing )
				{
					b_collision = true;
					indexClusters_reactive = index_reactive;
					// break;
				}
				if( distance2d(pt,cv::Point2f(predictor._pose.p)) < robot._radius )
				{
					b_break = true;
					break;
				}
			}
			if(b_collision == true)	break;
		}
		if(b_collision == true)	break;

		/*
		 * 	calculate the control error of straight line following
		 */
		motionControl control;
		control.straightlineFollowing(odometry, predictor._v, predictor._w);

		/*
		 * 	send the control command and update the pose of the robot
		 */
		double dx = predictor._v*std::cos(-predictor._pose.phi)*dt;			// attention there is a min
		double dy = predictor._v*std::sin(-predictor._pose.phi)*dt;
		predictor._pose.p.x = predictor._pose.p.x + dx;
		predictor._pose.p.y = predictor._pose.p.y + dy;
		predictor._pose.phi = predictor._pose.phi + predictor._w*dt;
		predictor._pose.phi = pi_to_pi(predictor._pose.phi);
		s_path = s_path + std::sqrt(dx*dx + dy*dy);
	}
	s_path = s_path + robot._disFollowing;
}

//// collision prediction for straight line following as the goal task
//// straight line following task is exacuted in the world frame
//// collision prediction is exacuted in the local frame (robot frame)
//// output the reactive obstacle and the protential collision distance
//void motionControl::collisioinPredictionStraightLineFollowing(	const LaserScan *las,
//																Navigation &navi,
//																const std::vector<std::vector<int> >  &Clusters,
//																const Pose &odometry,		// odometry in the world frame
//																const Robot &robot,
//																const float newVl,
//																const float newVa,
//																const double preavoidDistance,
//																bool &b_collision,
//																int &indexClusters_reactive,	// decide to reactive with which obstacle
//																double &s_path,					// protential collision distance)
//																std::vector<cv::Point2f> &trajectoryPrediction)
//{
//	// initial
//	s_path = 0;
//	b_collision = false;
//
//	double dt = 0.1;	// 0.1 second
//	/*
//	 * 	 inital the predictor
//	 */
//	Robot predictor(robot);
//	predictor._pose.p.x = 0;	// robot in the original in the local coordination
//	predictor._pose.p.y = 0;
//	predictor._pose.phi = 0;
//	if ( newVl < 0.2 )
//		predictor._v = 0.2;
//	else
//		predictor._v = newVl;
//	predictor._w = newVa;
//
//	trajectoryPrediction.clear();	// clear the predicted trajectory
//
//	/*
//	 * 	loop for motion prediction the same as the task straight line following
//	 */
//	bool b_break = false;
//	while( s_path < preavoidDistance )		// predict a given distance
//	{
//		/*
//		 *  collision verification
//		 *  output the index of reactive obstacle/clusters
//		 */
//		std::vector< std::vector<int> >::const_iterator iter_Clusters;
//		int index_reactive = 0;
//		for( iter_Clusters = Clusters.begin(); iter_Clusters < Clusters.end(); index_reactive++, iter_Clusters++ )
//		{
//			std::vector<int>::const_iterator iter_cluster;
//			for( iter_cluster = iter_Clusters->begin(); iter_cluster < iter_Clusters->end(); iter_cluster++ )
//			{
//				cv::Point2f pt;
//				navi.scan2CartesianInMeter(las,*iter_cluster,pt);
//				if( distance2d(pt,cv::Point2f(predictor._pose.p)) < robot._radius )
//				{
//					b_collision = true;
//					indexClusters_reactive = index_reactive;
//					// break;
//				}
////				if( distance2d(pt,cv::Point2f(predictor._pose.p)) < robot._radius )
////				{
////					b_break = true;
////					break;
////				}
//			}
//			if(b_collision == true)	break;
//		}
//		if(b_collision == true)	break;
//
//		/*
//		 * 	calculate the control error of straight line following
//		 */
//		motionControl control;
//		control.straightlineFollowing(odometry, predictor._v, predictor._w);
//
//		// record the predicted trajectory
//		trajectoryPrediction.push_back(cv::Point2f(predictor._pose.p.x, predictor._pose.p.y));
//
//		/*
//		 * 	send the control command and update the pose of the robot
//		 */
//		double dx = predictor._v*std::cos(predictor._pose.phi)*dt;
//		double dy = predictor._v*std::sin(predictor._pose.phi)*dt;
//		predictor._pose.p.x = predictor._pose.p.x + dx;
//		predictor._pose.p.y = predictor._pose.p.y + dy;
//		predictor._pose.phi = predictor._pose.phi - predictor._w*dt;	// attention there is a min for Lina
//		predictor._pose.phi = pi_to_pi(predictor._pose.phi);
//		s_path = s_path + std::sqrt(dx*dx + dy*dy);
//
//	}
//	s_path = s_path + robot._disFollowing;
//}

// collision prediction for straight line following as the goal task
// straight line following task is exacuted in the world frame
// collision prediction is exacuted in the local frame (robot frame)
// output the reactive obstacle and the protential collision distance
void motionControl::collisioinPredictionStraightLineFollowing(	const LaserScan *las,
																Navigation &navi,
																const std::vector<std::vector<int> >  &Clusters,
																const Pose &odometry,		// odometry in the world frame
																const Robot &robot,
																const float newVl,
																const float newVa,
																const double preavoidDistance,
																bool &b_collision,
																int &indexClusters_reactive,	// decide to reactive with which obstacle
																double &s_path,					// protential collision distance)
																std::vector<cv::Point2f> &trajectoryPrediction)
{
	// initial
	s_path = 0;				// accumulate
	double s_predict = 0;	// accumulate the predict length
	//b_collision = false;

	double dt = 0.1;	// 0.1 second
	/*
	 * 	 inital the predictor
	 */
	Robot predictor(robot);
	predictor._pose.p.x = 0;	// robot in the original in the local coordination
	predictor._pose.p.y = 0;
	predictor._pose.phi = 0;
	if ( newVl < 0.2 )
		predictor._v = 0.2;
	else
		predictor._v = newVl;
	predictor._w = newVa;
	Pose odometry_prediction;
	odometry_prediction.copyFrom(odometry);

	trajectoryPrediction.clear();	// clear the predicted trajectory

	/*
	 * 	loop for motion prediction the same as the task straight line following
	 */
	bool b_break = false;
	// while( s_path < preavoidDistance )		// predict a given distance
	while( s_predict < preavoidDistance )		// predict a given distance
	{
		/*
		 *  collision verification
		 *  output the index of reactive obstacle/clusters
		 */
		std::vector< std::vector<int> >::const_iterator iter_Clusters;
		int index_reactive = 0;
		for( iter_Clusters = Clusters.begin(); iter_Clusters < Clusters.end(); index_reactive++, iter_Clusters++ )
		{
			std::vector<int>::const_iterator iter_cluster;
			for( iter_cluster = iter_Clusters->begin(); iter_cluster < iter_Clusters->end(); iter_cluster++ )
			{
				cv::Point2f pt;
				navi.scan2CartesianInMeter(las,*iter_cluster,pt);
				if( distance2d(pt,cv::Point2f(predictor._pose.p)) <= robot._radius // define the close distance between the robot and ostacles
						&& b_collision == false )
				{
					b_collision = true;
					indexClusters_reactive = index_reactive;
					//std::cout << " distance : " << distance2d(pt,cv::Point2f(predictor._pose.p)) << "	" << indexClusters_reactive  << std::endl;
					// break;
				}
				else if ( b_collision == true && b_break == false
						  && distance2d(pt,cv::Point2f(predictor._pose.p)) < robot._radius // define the close distance between the robot and ostacles
						)
				{ b_break = true;
					indexClusters_reactive = index_reactive; }
			}
			// if(b_collision == true)	break;
		}
		// if(b_collision == true)	break;

		/*
		 * 	calculate the control error of straight line following
		 */
		motionControl control;
		control.straightlineFollowing(odometry_prediction, predictor._v, predictor._w);

		// record the predicted trajectory
		trajectoryPrediction.push_back(cv::Point2f(predictor._pose.p.x, predictor._pose.p.y));

		/*
		 * 	send the control command and update the pose of the robot, and updata the prediction odometry
		 */
		double dx = predictor._v*std::cos(predictor._pose.phi)*dt;
		double dy = predictor._v*std::sin(predictor._pose.phi)*dt;
		predictor._pose.p.x = predictor._pose.p.x + dx;
		predictor._pose.p.y = predictor._pose.p.y + dy;
		predictor._pose.phi = predictor._pose.phi + predictor._w*dt;	// attention there is a min for Lina
		predictor._pose.phi = pi_to_pi(predictor._pose.phi);

//		odometry_prediction.p.x = odometry_prediction.p.x + dx;
//		odometry_prediction.p.y = odometry_prediction.p.y + dy;
//		odometry_prediction.phi = odometry_prediction.phi + predictor._w*dt;
//		odometry_prediction.phi = pi_to_pi(odometry_prediction.phi);

		if ( b_collision == false )
			s_path = s_path + std::sqrt(dx*dx + dy*dy);
		s_predict = s_predict + std::sqrt(dx*dx + dy*dy);

	}
}

// collision detection using constant control for goal task during a sampling time
// the prediction is a circle arc in the current robot frame (local frame),
// therefore no global pose of the robot is need, which is different with the function collisioinPredictionStraightLineFollowing
// output the reactive obstacle and the protential collision distance
void motionControl::collisioinDetectionConstantControl( const LaserScan *las,
												Navigation &navi,
												const std::vector<std::vector<int> >  &Clusters,
												const Robot &robot,
												const float expectedVl,
												const float expectedVa,
												const double preavoidDistance,
												bool &b_collision,
												int &indexClusters_reactive,	// decide to reactive with which obstacle
												double &s_path,				// protential collision distance)
												std::vector<cv::Point2f> &trajectoryPrediction)
{
	// initial
	s_path = 0;
	// b_collision = false;

	double dt = 0.1;	// 0.1 second
	/*
	 * 	 inital the predictor
	 */
	Robot predictor(robot);
	predictor._pose.p.x = 0;	// robot in the original in the local coordination
	predictor._pose.p.y = 0;
	predictor._pose.phi = 0;
	if ( expectedVl < 0.2 )
		predictor._v = 0.2;
	else
		predictor._v = expectedVl;
	predictor._w = expectedVa;

	trajectoryPrediction.clear();	// clear the predicted trajectory

	/*
	 * 	loop for motion prediction the same as the task straight line following
	 */
	bool b_break = false;
	while( s_path < preavoidDistance )		// predict a given distance
	{
		/*
		 *  collision verification
		 *  output the index of reactive obstacle/clusters
		 */
		std::vector< std::vector<int> >::const_iterator iter_Clusters;
		int index_reactive = 0;
		for( iter_Clusters = Clusters.begin(); iter_Clusters < Clusters.end(); index_reactive++, iter_Clusters++ )
		{
			std::vector<int>::const_iterator iter_cluster;
			for( iter_cluster = iter_Clusters->begin(); iter_cluster < iter_Clusters->end(); iter_cluster++ )
			{
				cv::Point2f pt;
				navi.scan2CartesianInMeter(las,*iter_cluster,pt);
				if( distance2d(pt,cv::Point2f(predictor._pose.p)) < robot._radius // define the close distance between the robot and ostacles
						&& b_collision == false )								  // this part can be refined, for example considering the motion model to decide the dangerous zone
				{
					b_collision = true;
					indexClusters_reactive = index_reactive;
					// break;
				}
				else if ( b_collision == true
						  && distance2d(pt,cv::Point2f(predictor._pose.p)) < robot._radius + 0.5 // define the close distance between the robot and ostacles
						)
				{ indexClusters_reactive = index_reactive; }
			}
			// if(b_collision == true)	break;
		}
		// if(b_collision == true)	break;

		// record the predicted trajectory
		trajectoryPrediction.push_back(cv::Point2f(predictor._pose.p.x, predictor._pose.p.y));

		/*
		 * 	send the control command and update the pose of the robot
		 */
		double dx = predictor._v*std::cos(predictor._pose.phi)*dt;
		double dy = predictor._v*std::sin(predictor._pose.phi)*dt;
		predictor._pose.p.x = predictor._pose.p.x + dx;
		predictor._pose.p.y = predictor._pose.p.y + dy;
		predictor._pose.phi = predictor._pose.phi + predictor._w*dt;	// attention there is a min for Lina
		predictor._pose.phi = pi_to_pi(predictor._pose.phi);
		s_path = s_path + std::sqrt(dx*dx + dy*dy);
	}
}

// collision detection using constant control for goal task during a sampling time
// the prediction is a circle arc in the current robot frame (local frame),
// therefore no global pose of the robot is need, which is different with the function collisioinPredictionStraightLineFollowing
// output the reactive obstacle and the protential collision distance
void motionControl::collisioinPredictionGeneral( const LaserScan *las,
													Navigation &navi,		// wait to be replace by the distance function
													const std::vector<std::vector<int> >  &Clusters,
													const Pose &odometry,								// odometry in the world frame for reference path following
													const Robot &robot,
													const float expectedVl,
													const float expectedVa,
													const double preavoidDistance,
													const int    predictionMethod,
													bool &b_collision,
													int &indexClusters_reactive,
													double &s_path,
													std::vector<cv::Point2f> &trajectoryPrediction)
{
	if ( predictionMethod == PredictionMethod::STRAIGHT_LINE_FOLLOWING )
	{
		collisioinPredictionStraightLineFollowing( las, navi, Clusters, odometry, robot,
												   expectedVl, expectedVa, preavoidDistance, b_collision,
												   indexClusters_reactive, s_path, trajectoryPrediction);
	}
	else if ( predictionMethod == PredictionMethod::CONSTANT_CONTROL )
	{
		collisioinDetectionConstantControl( las, navi, Clusters, robot, expectedVl, expectedVa, preavoidDistance,
											b_collision, indexClusters_reactive, s_path, trajectoryPrediction);
	}
}

// fitting the boundary
void motionControl::obstacleBoundaryFitting( const LaserScan *las,
											   const std::vector<std::vector<int> >  &Clusters,
											   const std::vector<std::set<int> > PolygonalChains,
											   const int index_avoid,
											   BSplineCubic &curve)
{
	std::vector<std::set<int> >::const_iterator iter_PolygonalChains = PolygonalChains.begin() + index_avoid;

	std::vector<cv::Point2f> pts;	// restore the points of polygonalchain
	pts.reserve( (*iter_PolygonalChains).size() );
	std::set<int> ::const_iterator iter_polygonalchain;

	Navigation navi;
	navi.initLaser(1000, 0.006132813, 5000);

	for( iter_polygonalchain = iter_PolygonalChains->begin(); iter_polygonalchain != iter_PolygonalChains->end(); iter_polygonalchain++ )
	{
		cv::Point2f pt;
		navi.scan2Cartesian(las, *iter_polygonalchain, pt);
		pts.push_back(pt);
	}

	curve.calculateCubicCurve(pts, 0.05);
}

void motionControl::obstacleReactive( const LaserScan *las,
									   const std::vector<std::vector<int> >  &Clusters,
									   const std::vector<std::set<int> > PolygonalChains,
									   const int index_avoid,
									   const float dis_keep,
									   const Navigation &navi,
									   BSplineCubic &bspline,
									   int &indexLaser_near,
									   float &Vl,
									   float &Va)
{
	/*
	if ( PolygonalChains[index_avoid].size() > 1)	// use cubic curve fitting the boundary
	{
		// step 1: fitting the obstacle boundary using PolygonalChains
		obstacleBoundaryFitting( las, Clusters, PolygonalChains, index_avoid, bspline);

		// step 2: search for the nearest point on the fitting curve
		// search for the nearest point on the path
		int i_near = 0;     // for the nearest point
		double dis_nearcurve = 5000;
		double dis = 0;
		for (int i = 0; i < bspline.curve_BSpline.size(); i++)
		{
			cv::Point2f pt;
			pt.x = bspline.curve_BSpline[i].p.x;
			pt.y = bspline.curve_BSpline[i].p.y;

			dis = distance2d(pt, cv::Point2f(0, 0));
			if (dis < dis_nearcurve)
			{
				dis_nearcurve = dis;
				i_near = i;

			}
		}

		// step 3: transform the robot coordinate to the nearest curve point coordinate
		// set the nearest point as the relative coordinate
		Pose p_relative;
		p_relative.p.x = bspline.curve_BSpline[i_near].p.x;
		p_relative.p.y = bspline.curve_BSpline[i_near].p.y;
		p_relative.phi = bspline.curve_BSpline[i_near].phi;
		Transform2D trans2d(p_relative);		     // p_relative as the base
		// transform the robot coordinate (current robot frame) pose to the relative coordinate
		Pose p_robinrelative;
		p_robinrelative.p.x = 0;
		p_robinrelative.p.y = 0;
		p_robinrelative.phi = 0;
		trans2d.transform_to_relative(p_robinrelative);

		// step 4: calculate the error
		// angle calculation
		double alpha = p_robinrelative.phi;
		pi_to_pi(alpha);
		// delta calculation
		double delta = 0;
		double delta_abs = std::abs( distance2d(p_robinrelative.p, cv::Point2f(0, 0)) - dis_keep)/navi.getScaleLaser();  // keep 0.5m away from obstacle boundary
		if (p_robinrelative.p.y <= 0													// robot on the right side of the path
				&& distance2d(p_robinrelative, cv::Point2f(0, 0)) >= dis_keep)						// robot outside the defined area
		{ delta = -delta_abs; }
		if (p_robinrelative.p.y <= 0 													// robot on the right side of the path
				&& distance2d(p_robinrelative, cv::Point2f(0, 0)) < dis_keep) 						// robot inside the defined area
		{ delta = delta_abs; }
		if (p_robinrelative.p.y > 0 													// robot on the left side of the path
				&& distance2d(p_robinrelative, cv::Point2f(0, 0)) >= dis_keep) 						// robot outside the defined area
		{ delta = delta_abs; }
		if (p_robinrelative.p.y > 0														// robot on the left side of the path
				&& distance2d(p_robinrelative, cv::Point2f(0, 0)) < dis_keep) 						// robot inside the defined area
		{ delta = -delta_abs; }

		alpha = -bspline.curve_BSpline[i_near].phi;
		pi_to_pi(alpha);
		delta_abs = std::abs(distance2d(bspline.curve_BSpline[i_near].p, cv::Point2f(0, 0)) - dis_keep);
		if (distance2d(bspline.curve_BSpline[i_near].p, cv::Point2f(0, 0)) >= dis_keep)
		{ delta = -delta_abs; }
		else
		{
			delta = delta_abs;
		}

		// step 5: calculate the control law
		double chi = bspline.curve_BSpline[i_near].curvature;
		double k = 1.0;
		//delta = 0;
		chi = 0;    // chi is not good, give a threshold to decide when to use chi

		double chi_temp = 0;
		chi_temp = std::abs(1.0 / chi) + dis_keep;
		chi_temp = 1.0 / chi_temp;
		chi_temp = chi > 0 ? chi_temp : -chi_temp;

		Va = (float)(Vl * (k * delta + alpha + 2 * k * std::sin(alpha) - chi * std::cos(alpha)));

		//_newVa = (float)(_newVl * (k * delta + alpha + 2 * k * std::sin(alpha) - chi * std::cos(alpha)));
	}
	else										// use circle fitting the boundary
	{}
	*/
}

void motionControl::lowPassFilter(	const double k_LPF,			// the parameter of low pass filter
									const float expectedValue,		// the expected value
									float &filteredValue)			// the filtered value
{
	filteredValue = filteredValue + k_LPF * ( expectedValue - filteredValue );
}

// constructor
Robot::Robot() {};

Robot::Robot(	double	radius,
				Pose	pose,
				double	v,
				double	w,
				double	vMax,
				double	wMax,
				double	aMax,
				double	waMax,
				double	rMin,
				double	cMax,
				double	disFollowing,
				double	disReactive,
				double	disStop,
				double	vMaxOA,
				MotionState motionState
				)
{
	_radius 		= radius;
	_pose.copyFrom(pose);
	_v				= v;
	_w				= w;
	_vMax			= vMax;
	_wMax			= wMax;
	_aMax			= aMax;
	_waMax			= waMax;
	_rMin			= rMin;
	_cMax			= cMax;
	_disFollowing	= disFollowing;
	_disReactive	= disReactive;
	_disStop		= disStop;
	_vMaxOA			= vMaxOA;
	// motion state
	_motionState.motionStateCurrent			= motionState.motionStateCurrent;
	_motionState.motionStateRemembered 		= motionState.motionStateRemembered;
	_motionState.b_rotateFinding			= motionState.b_rotateFinding;
	_motionState.b_rotateFindingStart		= motionState.b_rotateFindingStart;
	_motionState.b_targetFindingDetection	= motionState.b_targetFindingDetection;
}

Robot::Robot(const Robot &robot)
{
	_radius 		= robot._radius;
	_pose.copyFrom(robot._pose);
	_v				= robot._v;
	_w				= robot._w;
	_vMax			= robot._vMax;
	_wMax			= robot._wMax;
	_aMax			= robot._aMax;
	_waMax			= robot._waMax;
	_rMin			= robot._rMin;
	_cMax			= robot._cMax;
	_disFollowing	= robot._disFollowing;
	_disReactive	= robot._disReactive;
	_disStop		= robot._disStop;
	_vMaxOA			= robot._vMaxOA;
	// motion state
	_motionState.motionStateCurrent			= robot._motionState.motionStateCurrent;
	_motionState.motionStateRemembered 		= robot._motionState.motionStateRemembered;
	_motionState.b_rotateFinding			= robot._motionState.b_rotateFinding;
	_motionState.b_rotateFindingStart		= robot._motionState.b_rotateFindingStart;
	_motionState.b_targetFindingDetection	= robot._motionState.b_targetFindingDetection;
}

// intinal function
void Robot::initial(	double	radius,
						Pose	pose,
						double	v,
						double	w,
						double	vMax,
						double	wMax,
						double	aMax,
						double	waMax,
						double	rMin,
						double	cMax,
						double	disFollowing,
						double	disReactive,
						double	disStop,
						double	vMaxOA,
						MotionState motionState)
{
	_radius 		= radius;
	_pose.copyFrom(pose);
	_v				= v;
	_w				= w;
	_vMax			= vMax;
	_wMax			= wMax;
	_aMax			= aMax;
	_waMax			= waMax;
	_rMin			= rMin;
	_cMax			= cMax;
	_disFollowing	= disFollowing;
	_disReactive	= disReactive;
	_disStop		= disStop;
	_vMaxOA			= vMaxOA;
	// motion state
	_motionState.motionStateCurrent			= motionState.motionStateCurrent;
	_motionState.motionStateRemembered 		= motionState.motionStateRemembered;
	_motionState.b_rotateFinding			= motionState.b_rotateFinding;
	_motionState.b_rotateFindingStart		= motionState.b_rotateFindingStart;
	_motionState.b_targetFindingDetection	= motionState.b_targetFindingDetection;
}

void Robot::initial(const Robot &robot)
{
	_radius 		= robot._radius;
	_pose.copyFrom(robot._pose);
	_v				= robot._v;
	_w				= robot._w;
	_vMax			= robot._vMax;
	_wMax			= robot._wMax;
	_aMax			= robot._aMax;
	_waMax			= robot._waMax;
	_rMin			= robot._rMin;
	_cMax			= robot._cMax;
	_disFollowing	= robot._disFollowing;
	_disReactive	= robot._disReactive;
	_disStop		= robot._disStop;
	_vMaxOA			= robot._vMaxOA;
	// motion state
	_motionState.motionStateCurrent			= robot._motionState.motionStateCurrent;
	_motionState.motionStateRemembered 		= robot._motionState.motionStateRemembered;
	_motionState.b_rotateFinding			= robot._motionState.b_rotateFinding;
	_motionState.b_rotateFindingStart		= robot._motionState.b_rotateFindingStart;
	_motionState.b_targetFindingDetection	= robot._motionState.b_targetFindingDetection;
}


/**
 *  visual servoing
 */
void VisualServoing::visualServoing_IJPU( const std::vector<cv::Point3f> &desired_corners_meter,		// visual features in the desired image
										  const std::vector<cv::Point3f> &scene_corners_current_meter, 	// visual features in the current image
										  const float Z,												// uniform depth information
										  const float lambd,											// gain
										  const float vl_vs,											// linear velocity
										  float	&va_vs)													// angular velocity
{
	// error and interaction matrix
	cv::Mat err(2*desired_corners_meter.size(), 1, CV_64FC1);
	cv::Mat Lsv = cv::Mat::zeros(2*desired_corners_meter.size(), 1, CV_64FC1);
	cv::Mat Lsw = cv::Mat::zeros(2*desired_corners_meter.size(), 1, CV_64FC1);
	for(int i = 0; i < scene_corners_current_meter.size(); i++)
	{
		err.at<double>(2*i,0) = scene_corners_current_meter[i].x - desired_corners_meter[i].x;
		err.at<double>(2*i+1,0) = scene_corners_current_meter[i].y - desired_corners_meter[i].y;

		double x = scene_corners_current_meter[i].x;
		double y = scene_corners_current_meter[i].y;
		//double Z = _Z_IJPU;
		Lsv.at<double>(2*i,0) = x/Z;
		Lsv.at<double>(2*i+1,0) = y/Z;
		Lsw.at<double>(2*i,0) = 1 + x*x;
		Lsw.at<double>(2*i+1,0) = x*y;
	}

	// for constant linear velocity
	cv::Mat temp(1, 1, CV_64FC1);
	temp = -Lsw.inv(cv::DECOMP_SVD)*(lambd*err+vl_vs*Lsv);
	va_vs = temp.at<double>(0,0);
}
