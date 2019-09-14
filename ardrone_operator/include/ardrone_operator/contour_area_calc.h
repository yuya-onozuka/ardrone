#ifndef CONTOUR_AREA_CALC_H_
#define CONTOUR_AREA_CALC_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class ContourAreaCalc
{
public:
    static double contourAreaCalc(cv::Mat& image);
};

#endif // CONTOUR_AREA_CALC_H_