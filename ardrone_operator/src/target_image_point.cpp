#include <ardrone_operator/target_image_point.h>

cv::Point2i TargetImagePoint::target_image_point(Eigen::Matrix3d camera_matrix, 
                                               Eigen::Matrix4d marker2camera_matrix, 
                                               Eigen::Vector4d marker_coordinate_target_point)
{
    Eigen::Vector4d camera_coordinate_point = marker2camera_matrix * marker_coordinate_target_point;
    
    Eigen::Vector3d screen_point = Eigen::Vector3d::Ones();
    
    for(int i = 0; i<2; i++){
        screen_point[i] = camera_coordinate_point[i] / camera_coordinate_point[2];
    }
    Eigen::Vector3d image_point = camera_matrix * screen_point;

    cv::Point2i target_image_point;
    target_image_point.x = image_point[0];
    target_image_point.y = image_point[1];
    // cv::circle(input_image, cv::Point(image_point[0], image_point[1]), 10, cv::Scalar(0,0,255), 3, 4);
    return target_image_point;
}
