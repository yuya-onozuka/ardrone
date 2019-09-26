#ifndef ESTIMATE_MARKER_POSE_H_
#define ESTIMATE_MARKER_POSE_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <vector>

class EstimateMarkerPose
{
public:
    static Eigen::Matrix4d estimateMarkerPose(cv::Mat& imput_image,
                                              cv::Mat& camera_matrix,
                                              cv::Mat& distortion_coefficients);
};

#endif // ESTIMATE_MARKER_POSE_H_