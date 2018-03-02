/*
 * motionControl.h
 *
 *  Created on: Oct 1, 2012
 *      Author: robot
 */

#ifndef MOTIONCONTROL_H_
#define MOTIONCONTROL_H_

#include <set>
#include "myUtilities.h"
#include "RobotLink.h"
#include "BSpline.h"
#include "Navigation.h"

// define the state of the robot
#define STATE_TARGET_DETECTION		0
#define STATE_TARGET_TRACKING		1
#define STATE_TARGET_FINDING		2
#define STATE_TARGET_FINDING_RANDOM 3
#define STATE_OBSTACLE_AVOIDANCE	4
#define STATE_STOP					5
#define STATE_END					6

// define prediction method
#define METHOD_STRAIGHT_LINE_FOLLOWING		0
#define METHOD_CONSTANT_CONTROL				1

class MotionState
{
public:
	static std::string get_State(const int n);

	enum {
		TARGET_DETECTION		= STATE_TARGET_DETECTION,		// SIFT detection
		TARGET_TRACKING			= STATE_TARGET_TRACKING,
		TARGET_FINDING			= STATE_TARGET_FINDING,
		TARGET_FINDING_RANDOM	= STATE_TARGET_FINDING_RANDOM,
		OBSTACLE_AVOIDANCE		= STATE_OBSTACLE_AVOIDANCE,
		STOP					= STATE_STOP,					// stop moving
		END						= STATE_END						// exit the project
	};

	int 	motionStateCurrent;									// the current motion state
	int 	motionStateRemembered;								// the remembered motion state (higher level mission)
	// for target finding
	bool	b_rotateFinding;									//
	bool 	b_rotateFindingStart;								//	initial rotate finding
	bool 	b_targetFindingDetection;							// if true, can do detection

};

class PredictionMethod
{
public:
	static std::string get_State(const int n);

	enum {
		STRAIGHT_LINE_FOLLOWING		= METHOD_STRAIGHT_LINE_FOLLOWING,
		CONSTANT_CONTROL			= METHOD_CONSTANT_CONTROL
	};
	int 	method;
};

class Robot
{
public:
	double	_radius;		// robot radius (0.25 m for Lina)
	Pose	_pose;			// pose (x,y,theta)
	double	_v;				// linear velocity (m/s)
	double	_w;				// angular velocity	(radian/s)
	double	_vMax;			// maximum linear velocity	(m/s)	0.4
	double	_wMax;			// maximum angular velocity	(radian/s) 20 deg/s
	double	_aMax;			// maximum linear acceleration (m/s^2) 0.4
	double	_waMax;			// maximum angular acceleration	(radian/s^2) 20 deg/s^2
	double	_rMin;			// minimum rotation radius	2 m
	double	_cMax;			// maximum curvature of rotation	0.5/m
	double 	_disFollowing;	// safe distance between the boundary of the robot and the boundary of the obstacle
	double	_disReactive;	// the zone to reactive with obstacle, distance is between the boundaries
	double 	_disStop;		// the distance to stop
	double	_vMaxOA;		// maximum velocity during obstacle avoidance

	MotionState _motionState;

	// constructor
	Robot();
	Robot(	double	radius,
			Pose	pose,
			double 	v,
			double 	w,
			double 	vMax,
			double 	wMax,
			double 	aMax,
			double 	waMax,
			double 	rMin,
			double 	cMax,
			double 	disFollowing,
			double 	disReactive,
			double 	disStop,
			double	vMaxOA,
			MotionState motionState);
	Robot (const Robot &robot);

	// assignment
	void initial( double	radius,
				  Pose		pose,
				  double 	v,
				  double 	w,
				  double 	vMax,
				  double 	wMax,
				  double 	aMax,
				  double 	waMax,
				  double 	rMin,
				  double 	cMax,
				  double 	disFollowing,
				  double 	disReactive,
				  double 	disStop,
				  double	vMaxOA,
				  MotionState motionState);
	void initial(const Robot &robot);

};

//class StereCamera
//{
//};
//
//class LaserRangefinder
//{};
//
//class RobotSystemConfiguration
//{};

class motionControl
{
public:
	void linearVelocity(const unsigned time_start,			// in milliseconds
						const unsigned time_stop,			// in milliseconds
						const unsigned time_current, 		// in milliseconds
						const bool	   b_start,
						const bool	   b_stop,
						const float    Vl_accumulate,			// accumulate
						const float    VlMax,
						float          &newVl);						// per second

	// linear profile (constant acceleration)
	void linearVelocity(const std::string 	motion_mode,
						const unsigned		time_0,				// in milliseconds
						const unsigned 		time_current, 		// in milliseconds
						const float			Vl0,
						const float			Vlexpected,
						const float    		VlMax,
						const float    		Vl_acceleration,	// accumulate
						float          		&newVl);			// per second, also the current velocity
	void linearVelocityBasedOnPotentialCollisionDistance(	const bool b_collision,
															const Robot &robot,			// constraints of the robot
															const double s_path,		// potential collision distance
															double &vl_expected,		// expected linear velocity
															bool &b_escape);			// obstacle escape condition

	void straightlineFollowing(const Pose odometry, double &Vl, double &Va);
	void obstacleAvoidenceOK( const LaserScan *las,
								const std::vector<std::vector<int> >  &Clusters,
								const float thresholdX_avoid,
								const float thresholdY_avoid,
								int &indexClusters_avoid,
								bool &b_avoid);

	// consider the primary task
	void obstacleAvoidenceOK( const LaserScan *las,
								const std::vector<std::vector<int> >  &Clusters,
								const Pose odometry,
								const float thresholdX_avoid,
								const float thresholdY_avoid,
								int &indexClusters_avoid,
								bool &b_avoid);

	// predictor for straight line following as the goal task
	void collisioinPredictionStraightLineFollowing( const LaserScan *las,
													Navigation &navi,
													const std::vector<std::vector<int> >  &Clusters,
													const Pose &odometry,								// odometry in the world frame
													const Robot &robot,
													const float newVl,
													const float newVa,
													const double preavoidDistance,
													bool &b_collision,
													int &indexClusters_avoid,
													double &s_path);									// protential collision distance

	// predictor for straight line following as the goal task with demo
	void collisioinPredictionStraightLineFollowing( const LaserScan *las,
													Navigation &navi,
													const std::vector<std::vector<int> >  &Clusters,
													const Pose &odometry,								// odometry in the world frame
													const Robot &robot,
													const float newVl,
													const float newVa,
													const double preavoidDistance,
													bool &b_collision,
													int &indexClusters_avoid,
													double &s_path,										// protential collision distance
													std::vector<cv::Point2f> &trajectoryPrediction);

	// predictor using constant control as the goal task
	void collisioinDetectionConstantControl( const LaserScan *las,
												Navigation &navi,
												const std::vector<std::vector<int> >  &Clusters,
												const Robot &robot,
												const float expectedVl,
												const float expectedVa,
												const double preavoidDistance,
												bool &b_collision,
												int &indexClusters_avoid,
												double &s_path,
												std::vector<cv::Point2f> &trajectoryPrediction);

	// general predictor
	void collisioinPredictionGeneral( const LaserScan *las,
										Navigation &navi,
										const std::vector<std::vector<int> >  &Clusters,
										const Pose &odometry,								// odometry in the world frame for reference path following
										const Robot &robot,
										const float expectedVl,
										const float expectedVa,
										const double preavoidDistance,
										const int    predictionMethod,
										bool &b_collisioin,
										int &indexClusters_avoid,
										double &s_path,
										std::vector<cv::Point2f> &trajectoryPrediction);

	// obstacle reactive
	void obstacleBoundaryFitting( const LaserScan *las,
								   const std::vector<std::vector<int> >  &Clusters,
								   const std::vector<std::set<int> > PolygonalChains,
								   const int index_avoid,
								   BSplineCubic &curve);

	void obstacleReactive( const LaserScan *las,
						   const std::vector<std::vector<int> >  &Clusters,
						   const std::vector<std::set<int> > PolygonalChains,
						   const int index_avoid,
						   const float dis_keep,
						   const Navigation &navi,
						   BSplineCubic &bspline,
						   int &indexLaser_near,
						   float &Vl,
						   float &Va);

	// low pass filter
	static void lowPassFilter( const double k_LPF_vl,			// the parameter of low pass filter
								const float expectedValue,		// the expected value
								float &filteredValue);			// the filtered value

	// prediction for collision detection
	bool    _b_clockwise;
	bool	_b_collision;
	int		_indexClusters_reactive;
	PredictionMethod _predictionMethod;
	double	_preavoidDistance;
	double	_s_path;			// arc length for collision
	bool 	_b_leave;
	cv::Point2f _nearPointOnBspline;
	std::vector<cv::Point2f> _trajectoryPrediction;
	float _dt;
};

class VisualServoing
{
public:

	// image jacobian points controller with uniform depths (IJPU)
	void visualServoing_IJPU( const std::vector<cv::Point3f> &desired_corners_meter,		// visual features in the desired image
							  const std::vector<cv::Point3f> &scene_corners_current_meter, 	// visual features in the current image
							  const float Z,												// uniform depth information
							  const float lambd,											// gain
							  const float vl_vs,											// linear velocity
							  float	&va_vs);												// angular velocity

	// IJPU
	float _Z_IJPU;			// depth information
	float _lambd_IJPU;		// gain
};

#endif /* MOTIONCONTROL_H_ */
