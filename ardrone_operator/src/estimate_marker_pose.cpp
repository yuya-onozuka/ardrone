#include <ardrone_operator/estimate_marker_pose.h>
#include <opencv2/aruco.hpp>

EstimateMarkerPose::EstimateMarkerPose()
{

}

EstimateMarkerPose::~EstimateMarkerPose()
{

}

std::vector<Eigen::Matrix4d> EstimateMarkerPose::estimateMarkersPose(cv::Mat& input_image,
                                                                     cv::Mat& camera_matrix,
                                                                     cv::Mat& distortion_coefficients,
                                                                     double marker_size)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);

    input_image.copyTo(draw_image_);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(input_image, dictionary, corners, ids);

    std::vector<cv::Vec3d> rotation_vectors, translation_vectors;
    std::vector<cv::Mat> rotation_matrixes(ids.size());

    std::vector<Eigen::Matrix4d> rt_transform_matrixes(ids.size(),Eigen::Matrix4d::Zero());

    // if at least one marker detected
    if (ids.size() > 0) 
    {
        cv::aruco::drawDetectedMarkers(draw_image_, corners, ids);
        
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors);
        
        for(int i=0; i<ids.size(); i++)
        {
            //rotation vectors to rotation matrixes
            cv::Rodrigues(rotation_vectors[i], rotation_matrixes[i]);

            // draw axis for each marker
            cv::aruco::drawAxis(draw_image_, camera_matrix, distortion_coefficients, rotation_vectors[i], translation_vectors[i], 0.1);

            //create transform_matrix
            cv::Vec3d translation_vector;
            cv::Mat rotation_matrix = (cv::Mat_<double>(3,3) << 0., 0., 0.,  0., 0., 0.,  0., 0., 0.);

            Eigen::Matrix4d rt_transform_matrix = Eigen::Matrix4d::Zero();

            translation_vector = translation_vectors[i];
            rotation_matrix = rotation_matrixes[i];
            
            rt_transform_matrix(3,3) = 1.;
            for(int j = 0; j < 3; j++)
            {
                for(int k = 0; k < 3; k++)
                {
                    rt_transform_matrix(j,k) = rotation_matrix.at<double>(j,k);
                }
            }
            for(int l = 0; l < 3; l++)
            {
                rt_transform_matrix(l,3) = translation_vector[l];
            }
            rt_transform_matrixes[i] = rt_transform_matrix;
            // rt_transform_matrixes[i] = rt_transform_matrix;
            // std::cout << rt_transform_matrixes[i] << std::endl;
            // std::cout << rotation_matrix << std::endl;
            // std::cout << rotation_matrixes[i] << std::endl;
            // std::cout << translation_vector << std::endl;
            // std::cout << translation_vectors[i] << std::endl;
        }
    }
    return rt_transform_matrixes;
}