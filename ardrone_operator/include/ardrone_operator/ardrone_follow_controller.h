#ifndef ARDRONE_FOLLOW_CONTROLLER_H_
#define ARDRONE_FOLLOW_CONTROLLER_H_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ardrone_autonomy/Navdata.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ardrone_operator/estimate_marker_pose.h>

class ArdroneFollowController
{
public:
    ArdroneFollowController();
    ~ArdroneFollowController();

private:
    void loadParameters();
    void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& navdata_msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
    void computeVelocity();

    //ros 
    ros::NodeHandle n_;
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher vel_pub_;
    image_transport::Publisher gray_image_pub_;
    image_transport::Publisher draw_image_pub_;

    ros::Subscriber navdata_sub_;
    image_transport::Subscriber image_sub_;

    std_msgs::Empty empty_msg_;
    geometry_msgs::Twist ardrone_vel_;

    bool is_flying_;
    unsigned int ardrone_state_;

    // variables for PID control
    int historical_deviation_num_;
    struct deviation_factors
    {
        double x;
        double y;
        double z;
        double angle;
        double time;
    };
    std::vector<deviation_factors> deviations_array_;

    // image
    cv::Mat input_image_;
    double state_marker_area_;
    cv::Point2i state_marker_point_;
    cv::Point2i target_marker_point_;

    // camera parameter
    Eigen::Matrix3d eigen_camera_matrix_;
    Eigen::VectorXd eigen_dist_coeffs_;

    // target parameter
    double marker_size_;
    double target_distance_;
    Eigen::Vector4d marker_coordinate_target_point_;
    
    // control parameter
    Eigen::Vector3d i_gain_;
    Eigen::Vector3d p_gain_;
    double i_angle_gain_;
    double p_angle_gain_;
    
    // estimate marker pose instance 
    std::unique_ptr<EstimateMarkerPose> emp_;
};

#endif // ARDRONE_FOLLOW_CONTROLLER_H_