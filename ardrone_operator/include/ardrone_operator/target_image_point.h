#ifndef TARGET_IMAGE_POINT_H_
#define TARGET_IMAGE_POINT_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <vector>

class TargetImagePoint
{
public:
    static cv::Point2i target_image_point(Eigen::Matrix3d camera_matrix, 
                                        Eigen::Matrix4d marker2camera_matrix, 
                                        Eigen::Vector4d marker_coordinate_target_point);
};

#endif // TARGET_IMAGE_POINT_H_