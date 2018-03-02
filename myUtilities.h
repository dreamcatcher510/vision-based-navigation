/*
 * myUtilities.h
 *
 *  Created on: Oct 30, 2012
 *      Author: robot
 */

#ifndef MYUTILITIES_H_
#define MYUTILITIES_H_

#include <opencv2/core/core.hpp>
#include <iostream>

#define PI 3.1415926535897932384626433832795

class Pose
{
public:
	cv::Point2f p;
	float 	phi;		// Radian with sign
	float	curvature;	// with sign

	Pose();
	Pose(const cv::Point2f pt, const float theta, const float curv);
	void copyFrom(const Pose &pose);
	void copyTo( Pose &pose );
};

inline
double pi_to_pi(double angle) {
	while ( angle < -PI  )
		angle += 2.*PI;
	while ( angle > PI )
		angle -= 2.*PI;
	return angle;
}

inline
float distance2d(cv::Point2f pt1, cv::Point2f pt2)
{
	float dx = pt1.x - pt2.x;
	float dy = pt1.y - pt2.y;
	return std::sqrt(dx*dx + dy*dy);
}

inline
long double distance2d(cv::Point2d pt1, cv::Point2d pt2)
{
	long double dx = pt1.x - pt2.x;
	long double dy = pt1.y - pt2.y;
	return std::sqrt((dx*dx + dy*dy));
}
inline
float distancePt2Line(cv::Point2f p, cv::Point2f Q1, cv::Point2f Q2)
{
	float norm = distance2d(Q1, Q2);
	float det = (Q1.x-Q2.x)*(p.y-Q1.y) - (Q1.y-Q2.y)*(p.x-Q1.x);
	float dis = std::abs(det) / norm;
	return dis;
}

class Transform2D {
public:
	Transform2D(const Pose& ref);

	void transform_to_relative(cv::Point2f &p);
	void transform_to_relative(Pose &p);
	void transform_to_global(cv::Point2f &p);
	void transform_to_global(Pose &p);

private:
	const Pose base;
	double c;
	double s;
};
inline
void Transform2D::transform_to_relative(cv::Point2f &p) {
	p.x -= base.p.x;
	p.y -= base.p.y;
	double t(p.x);
	p.x = p.x*c + p.y*s;
	p.y = p.y*c -   t*s;
}

inline
void Transform2D::transform_to_global(cv::Point2f &p) {
	double t(p.x);
	p.x = base.p.x + c*p.x - s*p.y;
	p.y = base.p.y + s*t   + c*p.y;
}

inline
void Transform2D::transform_to_relative(Pose &p) {
	transform_to_relative(p.p);
	p.phi= pi_to_pi(p.phi-base.phi);
}

inline
void Transform2D::transform_to_global(Pose &p) {
	transform_to_global(p.p);
	p.phi= pi_to_pi(p.phi+base.phi);
}

void momentObject();


#endif /* MYUTILITIES_H_ */
