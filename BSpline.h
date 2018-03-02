/*
 * BSpline.h
 *
 *  Created on: Sep 28, 2012
 *      Author: wenhao
 */

#ifndef BSPLINE_H_
#define BSPLINE_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "myUtilities.h"

class BSplineCubic
{
public:
	std::vector<cv::Point2f> pts_sample;			// points to be interpolated (sample data), all the passing points
	std::vector<cv::Point2f> pts_bsplineControl;	// B-Spline control points, with same number as sample points
	std::vector<cv::Point2f> pts_bezierControl;		// Bézier control points for the individual pieces (cubic Bézier 4 control points)
    std::vector<Pose> curve_BSpline;   		  		// result cubic curve interpolated

    // constraction function
    BSplineCubic();
    BSplineCubic( std::vector<cv::Point2f> pts,    // input the sample data points
                            double kk);            // input the partition length, for example 0.05

    void calculateCubicCurve(std::vector<cv::Point2f> pts,	// input the sample data points
            					double kk);					// input the partition length, for example 0.05

    //void drawBSplineCubicCurve(const LaserScan *las, const float width_area, const float height_area, const cv::Point2f ptObsNear, cv::Mat &I);

    int index_near;

private:
    int num_ptSample;				// the number of the sample data points, equal the number of bspline control points
    int num_ptBezierControl;		// Bézier control points for the individual pieces (cubic Bézier 4 control points),
    								// change with curve order
    int num_ptBezierCurve;			// the points of each Bézier curve 1/k
    double k;						// partition length, for example 0.05

    void calculateBSplineControlPoint();		   // input the sample data points, output the B-Spline control points
    void CubicCurve();    						   // input the B-Spline control points, output the result interpolated curve
    void CubicBezier( const std::vector<cv::Point2f> pts_control, std::vector<Pose> &curve );
};


#endif /* BSPLINE_H_ */
