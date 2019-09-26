#include <ardrone_operator/estimate_marker_pose.h>
#include <opencv2/aruco.hpp>

Eigen::Matrix4d estimateMarkerPose(cv::Mat& input_image,
                                   cv::Mat& camera_matrix,
                                   cv::Mat& distortion_coefficients)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);

    cv::Mat image_copy;
    input_image.copyTo(image_copy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(input_image, dictionary, corners, ids);

    std::vector<cv::Vec3d> rotation_vectors, translation_vectors;
    std::vector<cv::Mat> rotation_matrixes;
    // if at least one marker detected
    if (ids.size() > 0) 
    {
        cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
        
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors);
        
        for(int i=0; i<ids.size(); i++)
        {
            //rotation vectors to rotation matrixes
            cv::Rodrigues(rotation_vectors[i], rotation_matrixes[i]);
            // draw axis for each marker
            cv::aruco::drawAxis(image_copy, camera_matrix, distortion_coefficients, rotation_vectors[i], translation_vectors[i], 0.1);
        }
    }
    // cv::imshow("out", image_copy);

    //create transform_matrix
    Eigen::Matrix4d rt_transform_matrix = Eigen::Matrix4d::Zero();
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            rt_transform_matrix(i,j) = rotation_matrixes[0].at<double>(i,j);
        }
    }
    for(int i = 0; i < 3; i++)
    {
        rt_transform_matrix(i,3) = translation_vectors[0].at<double>(i,1);
    }
    return rt_transform_matrix;
}