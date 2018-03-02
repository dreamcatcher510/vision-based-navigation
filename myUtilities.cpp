/*
 * myUtilities.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: robot
 */

#include "myUtilities.h"
#include "math.h"

Transform2D::Transform2D( const Pose & ref) : base(ref)
{
	c = std::cos(ref.phi);
	s = std::sin(ref.phi);
}

// constructor
Pose::Pose() {}
Pose::Pose( const cv::Point2f pt, const float theta, const float curv )
{
	p.x 		= pt.x;
	p.y 		= pt.y;
	phi 		= theta;			// Radian with sign
	curvature	= curv;				// with sign
}

void Pose::copyFrom( const Pose &pose )
{
	p.x 		= pose.p.x;
	p.y 		= pose.p.y;
	phi 		= pose.phi;			// Radian with sign
	curvature	= pose.curvature;	// with sign
}

void Pose::copyTo( Pose &pose )
{
	pose.p.x 		= p.x;
	pose.p.y 		= p.y;
	pose.phi		= phi;					// Radian with sign
	pose.curvature 	= curvature;		// with sign
}

//
void momentObject()
{}
