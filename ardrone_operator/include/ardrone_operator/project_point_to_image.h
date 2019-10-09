#ifndef PROJECT_POINT_TO_IMAGE_H_
#define PROJECT_POINT_TO_IMAGE_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <vector>

class ProjectPointToImage
{
public:
    static cv::Point2i project_point_to_image(Eigen::Matrix3d camera_matrix, 
                                              Eigen::Matrix4d target2camera_rt_matrix, 
                                              Eigen::Vector4d target_coordinate_point);
};

#endif // PROJECT_POINT_TO_IMAGE_H_