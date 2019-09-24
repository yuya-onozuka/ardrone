#include <ardrone_operator/marker2camera_tf.h>

Eigen::Matrix4d Marker2CameraTransform::marker2camera_matrix(std::vector<cv::Point3f> marker_coordinate_points,
                                                             std::vector<cv::Point2f> camera_coordinate_points,
                                                             cv::Mat& camera_matrix,
                                                             cv::Mat& distortion_coefficients)
{
    cv::Mat rotation_vector = (cv::Mat_<double>(3,1) << 0., 0., 0.);
    cv::Mat tanslation_vector = (cv::Mat_<double>(3,1) << 0., 0., 0.);
    cv::Mat rotation_matrix = (cv::Mat_<double>(3,3) << 0., 0., 0.,  0., 0., 0.,  0., 0., 0.);

    //solve PnP problem
    cv::solvePnP(marker_coordinate_points, 
                 camera_coordinate_points, 
                 camera_matrix, 
                 distortion_coefficients, 
                 rotation_vector, 
                 tanslation_vector, 
                 false); 

    //rotation vector to rotation matrix
    cv::Rodrigues(rotation_vector, rotation_matrix);

    //create transform_matrix
    Eigen::Matrix4d rt_transform_matrix = Eigen::Matrix4d::Zero();
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            rt_transform_matrix(i,j) = rotation_matrix.at<double>(i,j);
        }
    }
    for(int i = 0; i < 3; i++)
    {
        rt_transform_matrix(i,3) = tanslation_vector.at<double>(i,1);
    }
    return rt_transform_matrix;
}