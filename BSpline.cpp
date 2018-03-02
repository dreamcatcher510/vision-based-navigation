/*
 * BSpline.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: wenhao
 */

#include "BSpline.h"
//#include <iostream>

BSplineCubic::BSplineCubic() {}

BSplineCubic::BSplineCubic( std::vector<cv::Point2f> pts,    // input the sample data points
                            double kk)            			 // input the partition length, for example 0.05
{
//	// method one: without convax decision
//	pts_sample = pts;					// assigns the sample data points, the sample points are the passing points
//	num_ptSample = pts.size();
//	// finish method one

	// method two: convax decision
	// select the convax points of the raw sample points
	float threshold_convax = 0;
	if( pts.size() == 2 )
	{
		pts_sample = pts;
	}
	else
	{
		pts_sample.push_back(pts[0]);
	    for( int i = 1; i < pts.size()-1; i++ )
	    {
	    	float atan_1 = std::atan2(pts[i].y-pts[i-1].y, pts[i].x-pts[i-1].x);
			float atan_2 = std::atan2(pts[i+1].y-pts[i].y, pts[i+1].x-pts[i].x);
			// if( atan_1 - atan_2 < 0 )
			if( atan_1 < atan_2 )
				pts_sample.push_back(pts[i]);
	    }
	    pts_sample.push_back( pts[pts.size()-1] );
	}
	num_ptSample = pts_sample.size();
	// finish method two

	pts_bsplineControl.reserve(num_ptSample);
	k = kk;

	num_ptBezierControl = 4;			// cubic Bézier 4 control points
	pts_bezierControl.reserve(num_ptBezierControl);
	num_ptBezierCurve = (int)(1 / k) + 1;

	calculateBSplineControlPoint();  // calculate the B-Spline control points
	CubicCurve();           		 // calculate the cubic curve
}

void BSplineCubic::calculateCubicCurve(std::vector<cv::Point2f> pts,	 // input the sample data points
        								double kk)						 // input the partition length, for example 0.05
{
	pts_sample.clear();
	pts_bsplineControl.clear();
	pts_bezierControl.clear();
    curve_BSpline.clear();

	pts_sample = pts;					// initialize the sample data points, the sample points are the passing points
	num_ptSample = pts_sample.size();
	pts_bsplineControl.reserve(num_ptSample);
	k = kk;

	num_ptBezierControl = 4;			// cubic Bézier 4 control points
	pts_bezierControl.reserve(num_ptBezierControl);
	num_ptBezierCurve = (int)(1 / k) + 1;

	calculateBSplineControlPoint();  // calculate the B-Spline control points
	CubicCurve();           		 // calculate the cubic curve
}

void BSplineCubic::calculateBSplineControlPoint()
{
	if ( num_ptSample == 2 )
	{
		pts_bsplineControl.push_back(pts_sample[0]);
		pts_bsplineControl.push_back(pts_sample[num_ptSample-1]);
	}

	if ( num_ptSample == 3 )
	{
		pts_bsplineControl.push_back(pts_sample[0]);
		// 6*S(1) = S(0) + 4*B(1) + S(2)
		double x, y;
		x = (6 * pts_sample[1].x - pts_sample[0].x - pts_sample[2].x) / 4;
		y = (6 * pts_sample[1].y - pts_sample[0].y - pts_sample[2].y) / 4;
		pts_bsplineControl.push_back(cv::Point2f(x,y));
		pts_bsplineControl.push_back(pts_sample[num_ptSample-1]);
	}

	if ( num_ptSample > 3 )
	{
	  // generate the 141 matrix
	  int n = num_ptSample - 2;				// the number of equations
	  cv::Mat matrix141(n,n,CV_64F);		// matrix except the first and last points B(0)=S(0), B(num_points-1)=S(num_points-1)
	  for (int i = 0; i < n; i++)     // generate the 141 matrix
	  {
		  for (int j = 0; j < n; j++)
		  {
			  matrix141.at<double>(i,j) = 0;
			  if (j == i)
			  {
				  matrix141.at<double>(i,j) = 4;
			  }
			  if (j == i - 1)
			  {
				  matrix141.at<double>(i,j) = 1;
			  }
			  if (j == i + 1)
			  {
				  matrix141.at<double>(i,j) = 1;
			  }
		  }
	  }
	  // calculate the inverse of 141 matrix
	  cv::Mat matrix141inverse = matrix141.inv(cv::DECOMP_LU);

	  // calculate the control points of B-Spline
	  // generate the right side of the 141 equations
	  cv::Mat equation141right_X(n,1,CV_64F);
	  cv::Mat equation141right_Y(n,1,CV_64F);
	  for (int i = 0; i < n; i++)		// for each equation (matrix row)
	  {
		  if (i == 0)     // the first equation, in the same line with B(1) or bspline_control[1]
		  {
			  equation141right_X.at<double>(i,0) = 6 * pts_sample[1].x - pts_sample[0].x;
			  equation141right_Y.at<double>(i,0) = 6 * pts_sample[1].y - pts_sample[0].y;
		  }
		  if (i == n-1)    // the last equation, in the same line with B(n) or bspline_control[num_points - 2]
		  {
			  equation141right_X.at<double>(i,0) = 6 * pts_sample[i+1].x - pts_sample[i + 2].x;
			  equation141right_Y.at<double>(i,0) = 6 * pts_sample[i+1].y - pts_sample[i + 2].y;
		  }
		  if (i != 0 && i != n-1)
		  {
			  equation141right_X.at<double>(i,0) = 6 * pts_sample[i+1].x;
			  equation141right_Y.at<double>(i,0) = 6 * pts_sample[i+1].y;
		  }
	  }

	  // calculate the control ponits except the first and the last
	  cv::Mat pts_controlX = matrix141inverse*equation141right_X;
	  cv::Mat pts_controlY = matrix141inverse*equation141right_Y;

	  // the control ponits
	  pts_bsplineControl.push_back(cv::Point2f(pts_sample[0]));		// the first one
	  for (int i = 0; i < n; i++)
	  {
		  pts_bsplineControl.push_back(cv::Point2f((float)pts_controlX.at<double>(i,0),(float)equation141right_Y.at<float>(i,0)));
	  }
	  pts_bsplineControl.push_back(cv::Point2f(pts_sample[num_ptSample-1]));	// the last one
	}
}

void BSplineCubic::CubicCurve()
{
	int tt = 0;

	for (int i = 1; i < num_ptSample; i++)    // generate num_points-1 Bézier curves
	{
		// generate the Bézier control points
		std::vector<cv::Point2f> pts_control;
		cv::Point2f pt;
		pts_control.push_back(pts_sample[i-1]);					  // P(0) = S(i-1)
		pt.x = (2.0 / 3) * pts_bsplineControl[i - 1].x + (1.0 / 3) * pts_bsplineControl[i].x; // P(1) = (2/3)B(i-1) + (1/3)B(i)
		pt.y = (2.0 / 3) * pts_bsplineControl[i - 1].y + (1.0 / 3) * pts_bsplineControl[i].y;
		pts_control.push_back(pt);
		pt.x = (1.0 / 3) * pts_bsplineControl[i - 1].x + (2.0 / 3) * pts_bsplineControl[i].x; // P(2) = (1/3)B(i-1) + (2/3)B(i)
		pt.y = (1.0 / 3) * pts_bsplineControl[i - 1].y + (2.0 / 3) * pts_bsplineControl[i].y;
		pts_control.push_back(pt);
		pts_control.push_back(pts_sample[i]);				       // P(3) = S(i)

		std::vector<Pose> BezierCurve;
		BezierCurve.reserve(num_ptBezierCurve);
		CubicBezier( pts_control, BezierCurve);

		// assign all the points of Bézier to the m_BSplineCurve
		if (i == 1)
		{
		   for (int ii = 0; ii < num_ptBezierCurve; ii++)
		   {
			   curve_BSpline.push_back(BezierCurve[ii]);
		   }
		}
		else
		{
		   for (int ii = 0; ii < num_ptBezierCurve; ii++)
		   {
			   curve_BSpline.push_back(BezierCurve[ii]);
		   }
		}
	}
}

void BSplineCubic::CubicBezier(const std::vector<cv::Point2f> pts_control, std::vector<Pose> &curve)
{
	double a0, a1, a2, a3;		//	coordinate of contorl points on x-axis
	double b0, b1, b2, b3;      //  coordinate of control points on y-axis
	double cx0, cx1, cx2, cx3;	//	coefficients of the polynomial of t with respect to x
	double cy0, cy1, cy2, cy3;  //	coefficients of the polynomial of t with respect to y
	double dx, dy;				//	first derivatives
	double dx0, dx1, dx2;		//	first derivatives of the polynomial of t with respect to x
	double dy0, dy1, dy2;       //	first derivatives of the polynomial of t with respect to y
	double d2x, d2y;			//	second derivatives
	double d2x0, d2x1;    //	second derivatives of the polynomial of t with respect to x
	double d2y0, d2y1;    //	second derivatives of the polynomial of t with respect to y
	double t;

	// coordinate of control points
	std::vector<cv::Point2f>::const_iterator iter = pts_control.begin();
	a0 = (*iter).x;
	b0 = (*iter).y;
	iter++;
	a1 = (*iter).x;
	b1 = (*iter).y;
	iter++;
	a2 = (*iter).x;
	b2 = (*iter).y;
	iter++;
	a3 = (*iter).x;
	b3 = (*iter).y;

	// coefficients of the polynomial of t P(t) = ...
	// x(t) = cx3*t^3 + cx2*t^2 + cx1*t + cx0
	// y(t) = cy3*t^3 + cy2*t^2 + cy1*t + cy0
	cx3 = -a0 + 3 * (a1 - a2) + a3;
	cy3 = -b0 + 3 * (b1 - b2) + b3;
	cx2 = 3 * (a0 - 2 * a1 + a2);
	cy2 = 3 * (b0 - 2 * b1 + b2);
	cx1 = 3 * (a1 - a0);
	cy1 = 3 * (b1 - b0);
	cx0 = a0;
	cy0 = b0;

	// coefficients of first derivatives of the polynomial of t
	// dx(t) = 3*cx3*t^2 + 2*cx2*t + cx1
	// dy(t) = 3*cy3*t^2 + 2*cy2*t + cy1
	dx2 = 3 * cx3;
	dy2 = 3 * cy3;
	dx1 = 2 * cx2;
	dy1 = 2 * cy2;
	dx0 = cx1;
	dy0 = cy1;

	// coefficients of second derivatives of the polynomial of t
	// d2x(t) = 6*cx3*t + 2*cx2
	// d2y(t) = 6*cy3*t + 2*cy2
	d2x1 = 6 * cx3;
	d2y1 = 6 * cy3;
	d2x0 = 2 * cx2;
	d2y0 = 2 * cy2;

	Pose p_curve;
	p_curve.p.x = a0;
	p_curve.p.y = b0;
	p_curve.phi = std::atan2(dy0, dx0);
	p_curve.curvature = (dx0 * d2y0 - dy0 * d2x0) / std::pow(std::pow(dx0, 2) + std::pow(dy0, 2), (3.0 / 2.0));

	for (int i = 1; i <= num_ptBezierCurve; i++)
	{
		t = i * k;
		// coordinate
		p_curve.p.x = ((cx3 * t + cx2) * t + cx1) * t + cx0;
		p_curve.p.y = ((cy3 * t + cy2) * t + cy1) * t + cy0;

		// first derivatives for theta
		dx = dx2 * std::pow(t, 2) + dx1 * t + dx0;
		dy = dy2 * std::pow(t, 2) + dy1 * t + dy0;
		p_curve.phi = std::atan2(dy, dx);    // Radian

		// second derivatives for curvature
		d2x = d2x1 * t + d2x0;
		d2y = d2y1 * t + d2y0;
		p_curve.curvature = (dx * d2y - dy * d2x) / std::pow(std::pow(dx, 2) + std::pow(dy, 2), (3.0 / 2.0));
		curve.push_back(p_curve);
	}
}


