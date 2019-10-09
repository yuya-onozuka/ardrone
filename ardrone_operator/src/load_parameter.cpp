#include <ardrone_operator/ardrone_follow_controller.h>

void ArdroneFollowController::loadParameters()
{
    ros::NodeHandle private_nh("~");

    std::vector<double> camera_mat;
    std::vector<double> dist_coeffs;
    std::vector<double> target_point;
    std::vector<double> p_gain;
    std::vector<double> i_gain;

    private_nh.getParam("camera_matrix/data", camera_mat);
    private_nh.getParam("distortion_coefficients/data", dist_coeffs);
    private_nh.getParam("target/marker_size", marker_size_);
    private_nh.getParam("target/target_distance", target_distance_);
    private_nh.getParam("target/target_point", target_point);
    private_nh.getParam("controller/p_gain", p_gain);
    private_nh.getParam("controller/i_gain", i_gain);
    private_nh.getParam("controller/p_angle_gain", p_angle_gain_);
    private_nh.getParam("controller/i_angle_gain", i_angle_gain_);
    private_nh.getParam("controller/historical_deviation_number", historical_deviation_num_);

    // std::vectorからEigen::Matrix, Eigen::Vectorに変換
    for(int row = 0; row < 3; row++)
    {
        for(int col = 0; col < 3; col++)
        {
            eigen_camera_matrix_(row, col) = camera_mat[3*row + col];
        }
    }

    for(int i = 0; i < dist_coeffs.size(); i++)
    {
        eigen_dist_coeffs_.resize(dist_coeffs.size());
        eigen_dist_coeffs_[i] = dist_coeffs[i];
    }

    for(int i = 0; i < target_point.size(); i++)
    {
        marker_coordinate_target_point_[i] = target_point[i];
    }

    for(int i = 0; i < p_gain.size(); i++)
    {
        p_gain_[i] = p_gain[i];
        i_gain_[i] = i_gain[i];
    }
}