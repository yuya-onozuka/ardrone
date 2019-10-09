#include <ardrone_operator/project_point_to_image.h>

cv::Point2i ProjectPointToImage::project_point_to_image(Eigen::Matrix3d camera_matrix, 
                                                        Eigen::Matrix4d target2camera_rt_matrix, 
                                                        Eigen::Vector4d target_coordinate_point)
{
    Eigen::Vector4d camera_coordinate_point = target2camera_rt_matrix * target_coordinate_point;
    
    Eigen::Vector3d screen_point = Eigen::Vector3d::Ones();
    
    for(int i = 0; i<2; i++){
        screen_point[i] = camera_coordinate_point[i] / camera_coordinate_point[2];
    }
    Eigen::Vector3d image_point = camera_matrix * screen_point;

    cv::Point2i projected_point;
    projected_point.x = image_point[0];
    projected_point.y = image_point[1];
    // cv::circle(input_image, cv::Point(image_point[0], image_point[1]), 10, cv::Scalar(0,0,255), 3, 4);
    return projected_point;
}
