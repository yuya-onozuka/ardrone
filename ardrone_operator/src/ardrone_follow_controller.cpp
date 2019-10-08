#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ardrone_operator/calc_area_in_contour.h>
#include <ardrone_operator/red_extraction.h>
#include <ardrone_operator/estimate_marker_pose.h>
#include <ardrone_operator/target_image_point.h>


class ArdroneFollowController
{
public:
    ArdroneFollowController();
    ~ArdroneFollowController();

private:
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void computeVelocity();

    //ros 
    ros::NodeHandle n_;
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher vel_pub_;
    image_transport::Publisher gray_image_pub_;
    image_transport::Publisher draw_image_pub_;
    image_transport::Subscriber image_sub_;
    std_msgs::Empty empty_msg_;
    geometry_msgs::Twist ardrone_vel_;

    bool takeoff_flag_;

    cv::Mat input_image_;
    
    int deviation_number_threshold_;
    struct deviation_factors
    {
        double x;
        double y;
        double z;
        double angle;
        double time;
    };
    std::vector<deviation_factors> deviations_array_;

    double state_marker_area_;
    cv::Point2i state_marker_point_;
    cv::Point2i target_marker_point_;

    // camera parameter
    Eigen::Matrix3d eigen_camera_matrix_;
    Eigen::VectorXd eigen_dist_coeffs_;

    Eigen::Vector4d marker_coodinate_target_point_;

    //control parameter
    Eigen::Vector3d i_gain_;
    Eigen::Vector3d p_gain_;
    double i_angle_gain_;
    double p_angle_gain_;
    double target_distance_;
    
    // estimate marker pose instance 
    std::unique_ptr<EstimateMarkerPose> emp_;
};

ArdroneFollowController::ArdroneFollowController()
{
    //ros pub sub
    image_transport::ImageTransport it(n_);
    image_sub_ = it.subscribe("/ardrone/front/image_raw", 1, &ArdroneFollowController::imageCallback, this);
    gray_image_pub_ = it.advertise("/red_detection_image", 10);
    draw_image_pub_ = it.advertise("/estimate_pose_image",10);
    takeoff_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    land_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/land",1);
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    takeoff_flag_ = false;

    deviation_number_threshold_ = 15;

    // camera parameter
    eigen_camera_matrix_ << 553.627733, 0.000000, 317.448667, 
                            0.000000, 550.558377, 122.189254, 
                            0.000000, 0.000000, 1.000000;
    eigen_dist_coeffs_.resize(5);
    eigen_dist_coeffs_ << -0.519086, 0.331704, 0.013667, 0.002975, 0.000000;

    marker_coodinate_target_point_ << 0., 0., 0., 1.;

    // control parameter(drone coordinate)
    i_gain_ << 0.6, 0.000, 0.002;
    p_gain_ << 0.3, 0.000, 0.002;
    i_angle_gain_ = 1.8;
    p_angle_gain_ = 1.0;
    target_distance_ = 0.7;

    emp_.reset(new EstimateMarkerPose());
}

ArdroneFollowController::~ArdroneFollowController()
{
}

void ArdroneFollowController::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	try
    {
        input_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	//error
	catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    cv::eigen2cv(eigen_camera_matrix_, camera_matrix);
    cv::eigen2cv(eigen_dist_coeffs_, distortion_coefficients);

    std::vector<Eigen::Matrix4d> rt_matrixes = emp_->estimateMarkersPose(input_image_, camera_matrix, distortion_coefficients, 0.15);

    if(rt_matrixes.size()>0)
    {
        // targetはひとつだけ
        state_marker_point_ = TargetImagePoint::target_image_point(eigen_camera_matrix_, rt_matrixes[0], marker_coodinate_target_point_);
        cv::circle(emp_->draw_image_, cv::Point(state_marker_point_.x, state_marker_point_.y), 10, cv::Scalar(0,0,255), 3, 4);
    }

    //赤色を抽出して白黒画像に変換
    cv::Mat red_extract_image = RedExtraction::extractRedFrom(input_image_);
    //最も面積が大きい輪郭の面積
    state_marker_area_ = CalcAreaInContour::calcAreaInContour(red_extract_image);
    // 最も面積が大きい輪郭の重心
    // state_marker_pos_ = CalcAreaInContour::calcCentroidInContour(red_extract_image);

    int height = input_image_.rows;
    int width = input_image_.cols;
    target_marker_point_.x = width / 2;
    target_marker_point_.y = height / 2;

    double occupied_area_ratio_red = state_marker_area_ / (width * height);
    
    ROS_INFO("occupied_area_red = %lf",state_marker_area_);
    ROS_INFO("occupied_area_ratio_red = %lf",occupied_area_ratio_red);
    ROS_INFO("ardrone_vel_x = %lf",ardrone_vel_.linear.x);
    ROS_INFO("ardrone_vel_y = %lf",ardrone_vel_.linear.y);
    ROS_INFO("ardrone_vel_z = %lf",ardrone_vel_.linear.z);

 	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_extract_image).toImageMsg();
 	gray_image_pub_.publish(img_msg);

    sensor_msgs::ImagePtr draw_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", emp_->draw_image_).toImageMsg();
    draw_image_pub_.publish(draw_img_msg);

    //過去5個の平均
    deviation_factors new_factor;

    if(rt_matrixes.size() > 0)
    {
        new_factor.x = target_marker_point_.x - state_marker_point_.x;
        new_factor.y = target_marker_point_.y - state_marker_point_.y;
        new_factor.z = rt_matrixes[0](2,3) - target_distance_;
        new_factor.angle = atan(-rt_matrixes[0](0,3)/(rt_matrixes[0](2,3) + 0.00001));
        new_factor.time = ros::Time::now().toSec();
    }
    else
    {
        new_factor.x = 0;
        new_factor.y = 0;
        new_factor.z = 0;
        new_factor.angle = 0;
        new_factor.time = ros::Time::now().toSec();
    }

    if(deviations_array_.size() == deviation_number_threshold_)
    {
        for (int i = 0; i<deviation_number_threshold_ - 1; i++)
        {
            deviations_array_[i] = deviations_array_[i+1];
        }
        deviations_array_[deviation_number_threshold_ - 1] = new_factor;
    }
    else
    {
         deviations_array_.push_back(new_factor);
    }
    
    
    if(takeoff_flag_==true)
    {
        ArdroneFollowController::computeVelocity();      
        vel_pub_.publish(ardrone_vel_);
    }

    if(occupied_area_ratio_red > 0.3 && takeoff_flag_ == false)
    {
        takeoff_pub_.publish(empty_msg_);
        takeoff_flag_ = true;
        ROS_INFO("takeoff");
    }
}

void ArdroneFollowController::computeVelocity()
{
    //PID制御したい
    if(deviations_array_.size()>0)
    {
        deviation_factors i_factors;
        deviation_factors p_factors;
        deviation_factors d_factors;
        i_factors.x = 0.;
        i_factors.y = 0.;
        i_factors.z = 0.;
        i_factors.angle = 0.;

        p_factors = deviations_array_[deviations_array_.size() - 1];

        for(int i = 0 ; i<deviations_array_.size() ;i++)
        {
            i_factors.x += deviations_array_[i].x;
            i_factors.y += deviations_array_[i].y;
            i_factors.z += deviations_array_[i].z;
            i_factors.angle = deviations_array_[i].angle;
        }

        i_factors.x = i_factors.x/deviations_array_.size();
        i_factors.y = i_factors.y/deviations_array_.size();
        i_factors.z = i_factors.z/deviations_array_.size();
        i_factors.angle = i_factors.angle/deviations_array_.size();
         
        ardrone_vel_.linear.x =  i_gain_[0] *i_factors.z +p_gain_[0] * p_factors.z;
        ardrone_vel_.linear.y = i_gain_[1] *i_factors.x + p_gain_[1] * p_factors.x;
        ardrone_vel_.linear.z = i_gain_[2] *i_factors.y + p_gain_[2] * p_factors.y;
        ardrone_vel_.angular.z = i_factors.angle* i_angle_gain_ + p_angle_gain_ * p_factors.angle;

    }
}

int main(int argc, char** argv)
{
 	ros::init (argc, argv, "ardrone_follow_controller");
    ros::NodeHandle n;
    ros::Rate rate(10);

    ArdroneFollowController follow;

    ros::spin();
 	return 0;
}