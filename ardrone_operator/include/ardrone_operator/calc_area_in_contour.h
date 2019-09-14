#ifndef CALC_AREA_IN_CONTOUR_H_
#define CALC_AREA_IN_CONTOUR_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <vector>

class CalcAreaInContour
{
public:
    static double calcAreaInContour(cv::Mat& image);
    static Eigen::Vector2d calcCentroidInContour(cv::Mat& image);

private:
    std::vector<cv::Point> points_in_contour_;
    bool calc_flag_;
};

#endif // CALC_AREA_IN_CONTOUR_H_