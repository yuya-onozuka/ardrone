#include <ardrone_operator/contour_area_calc.h>
#include <iostream>
#include <vector>

double ContourAreaCalc::contourAreaCalc(cv::Mat& image)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    double max_area = 0;
    int max_area_contour = -1;
    for(int j = 0; j < contours.size(); j++)
    {
        double area = contourArea(contours.at(j));
        if(max_area < area)
        {
            max_area = area;
            max_area_contour = j;
        }
    }
    return max_area;
}