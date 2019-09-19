#include <ardrone_operator/calc_area_in_contour.h>
#include <iostream>
#include <vector>

double CalcAreaInContour::calcAreaInContour(cv::Mat& image)
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

Eigen::Vector2d CalcAreaInContour::calcCentroidInContour(cv::Mat& image)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    double max_area = 0;
    int max_area_contour = -1;
    for(int k = 0; k < contours.size(); k++)
    {
        double area = contourArea(contours.at(k));
        if(max_area < area)
        {
            max_area = area;
            max_area_contour = k;
        }
    }
    int points_num = contours.at(max_area_contour).size();
    Eigen::Vector2d centroid(0., 0.);
    for(int i; i < points_num; i++)
    {
        centroid[0] += contours.at(max_area_contour).at(i).x;
        centroid[1] += contours.at(max_area_contour).at(i).y;
    }
    centroid[0] /= points_num;
    centroid[1] /= points_num;
    // centroid[0] = (contours.at(max_area_contour).at(0).x + contours.at(max_area_contour).at(points_num -1).x)/2;
    // centroid[1] = (contours.at(max_area_contour).at(0).y + contours.at(max_area_contour).at(points_num -1).y)/2;

    return centroid;
}
