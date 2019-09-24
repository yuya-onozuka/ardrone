#ifndef MARKER2CAMERA_TF_H_
#define MARKER2CAMERA_TF_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <vector>

class Marker2CameraTransform
{
public:
    static Eigen::Matrix4d marker2camera_matrix(std::vector<cv::Point3f> marker_coordinate_points,
                                                std::vector<cv::Point2f> camera_coordinate_points,
                                                cv::Mat& camera_matrix,
                                                cv::Mat& distortion_coefficients);
};

#endif // MARKER2CAMERA_TF_H_