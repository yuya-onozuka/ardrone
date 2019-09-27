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
    EstimateMarkerPose();
    ~EstimateMarkerPose();

    std::vector<Eigen::Matrix4d> estimateMarkersPose(cv::Mat& imput_image,
                                                     cv::Mat& camera_matrix,
                                                     cv::Mat& distortion_coefficients);

    cv::Mat draw_image_;
};

#endif // ESTIMATE_MARKER_POSE_H_