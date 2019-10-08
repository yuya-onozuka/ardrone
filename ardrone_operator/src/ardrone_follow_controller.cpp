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
    bool time_start_flag_;
    double pre_time_;
    

private:
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void computeVelocity();
    void hoveringGradually();

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

    cv::Mat input_image_;
    double state_marker_area_;
    
    double now_time_;
    double time_duration_threshold_ ;

    //double average_deviation_x_image_;
    //double average_deviation_y_image_;

    int time_counter_;
    int deviation_number_threshold_;


    struct deviation_factors
    {
        double x;
        double y;
        double angle;
        double time;


    };

    std::vector<deviation_factors> deviations_array_;



    cv::Point2i state_marker_point_;
    cv::Point2i target_marker_point_;

    Eigen::Matrix3d eigen_camera_matrix_;
    Eigen::VectorXd eigen_dist_coeffs_;

    Eigen::Vector4d marker_coodinate_target_point_;

    bool takeoff_flag_;
    bool land_flag_;
    bool calc_finished_flag_;
    
    // estimate marker pose instance 
    std::unique_ptr<EstimateMarkerPose> emp_;
};

ArdroneFollowController::ArdroneFollowController()
{
    image_transport::ImageTransport it(n_);
    image_sub_ = it.subscribe("/ardrone/front/image_raw", 1, &ArdroneFollowController::imageCallback, this);
    gray_image_pub_ = it.advertise("/image_contour", 10);
    draw_image_pub_ = it.advertise("/estimate_pose_image",10);
    takeoff_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    land_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/land",1);
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    land_flag_ = false;
    time_duration_threshold_ = 1.;
    deviation_number_threshold_ = 5;

    time_counter_ = 0;

    eigen_camera_matrix_ << 553.627733, 0.000000, 317.448667, 
                            0.000000, 550.558377, 122.189254, 
                            0.000000, 0.000000, 1.000000;
    eigen_dist_coeffs_.resize(5);
    eigen_dist_coeffs_ << -0.519086, 0.331704, 0.013667, 0.002975, 0.000000;

    marker_coodinate_target_point_ << 0., 0., 0., 1.;

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

    now_time_ = ros::Time::now().toSec();
    if (now_time_ - pre_time_ > time_duration_threshold_)
    {
        pre_time_ = now_time_;
        time_counter_ = 0;
        //average_deviation_x_image_ = 0;
        //average_deviation_y_image_ = 0;
    }

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    cv::eigen2cv(eigen_camera_matrix_, camera_matrix);
    cv::eigen2cv(eigen_dist_coeffs_, distortion_coefficients);

    std::vector<Eigen::Matrix4d> rt_matrixes = emp_->estimateMarkersPose(input_image_, camera_matrix, distortion_coefficients);
    ROS_INFO("rt_matrix_size = %d",rt_matrixes.size());
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

    // ROS_INFO("take_off_flag = %s",takeoff_flag_);
    // if(calc_finished_flag_)
    // {
    //     calc_finished_flag_ = false;
    //     state_marker_pos_ = CalcAreaInContour::calcCentroidInContour(red_extract_image);
    //     calc_finished_flag_ = true;
    // }
    // state_marker_pos_ = CalcAreaInContour::calcCentroidInContour(red_extract_image);

    int height = input_image_.rows;
    int width = input_image_.cols;
    target_marker_point_.x = width / 2;
    target_marker_point_.y = height / 2;

    double occupied_area_ratio_red = state_marker_area_ / (width * height);
    
    ROS_INFO("unko");
    ROS_INFO("occupied_area_red = %lf",state_marker_area_);
    ROS_INFO("occupied_area_ratio_red = %lf",occupied_area_ratio_red);
    // ROS_INFO("x = %lf",state_marker_pos_[0]);
    // ROS_INFO("y = %lf",state_marker_pos_[1]);
    ROS_INFO("height = %d",height);
    ROS_INFO("width = %d",width);
    ROS_INFO("ardrone_vel_x = %lf",ardrone_vel_.linear.x);
    ROS_INFO("ardrone_vel_y = %lf",ardrone_vel_.linear.y);
    ROS_INFO("ardrone_vel_z = %lf",ardrone_vel_.linear.z);
    ROS_INFO("time = %lf",pre_time_);
    ROS_INFO("time_counter = %d",time_counter_);

 	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_extract_image).toImageMsg();
 	gray_image_pub_.publish(img_msg);

    sensor_msgs::ImagePtr draw_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", emp_->draw_image_).toImageMsg();
    draw_image_pub_.publish(draw_img_msg);

    time_counter_ += 1;

    //過去5個の平均
    deviation_factors new_factor;

    if(rt_matrixes.size() > 0)
    {
        new_factor.x = target_marker_point_.x - state_marker_point_.x;
        new_factor.y = target_marker_point_.y - state_marker_point_.y;
        new_factor.angle = atan(-rt_matrixes(0,3)/(rt_matrixes(2,3) + 0.00001));
        new_factor.time = ros::Time::now.toSec();
        

    }
    else
    {
        new_factor.x = 0;
        new_factor.y = 0;
        new_factor.angle = rt_matrixes(0,3)/(rt_matrixes(2,3) + 0.00001);
        new_factor.time = ros::Time::now.toSec();
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
           
        if(rt_matrixes.size() > 0)
        {
            ArdroneFollowController::computeVelocity();
        }
        else if(rt_matrixes.size() == 0)
        {
            ArdroneFollowController::computeVelocity();
            //ArdroneFollowController::hoveringGradually();
        }
        
        vel_pub_.publish(ardrone_vel_);
        ROS_INFO("unko_vel");
    }

    if(occupied_area_ratio_red > 0.3 && takeoff_flag_ == NULL)
    {
        takeoff_pub_.publish(empty_msg_);
        takeoff_flag_ = true;
        ROS_INFO("unko_takeoff");
    }
}

void ArdroneFollowController::computeVelocity()
{
    //PI制御したい
    Eigen::Vector3d i_gain(0.00005, 0.001, 0.001);
    double i_angle_gain = 0.;
    
    Eigen::Vector3d p_gain(0.00000, 0.000, 0.000);
    double p_angle_gain = 0.;

    double target_maker_area = 10000;

    if(deviations_array_.size()>0)
    {
        double i_dev_x = 0.;
        double i_dev_y = 0.;
        double i_dev_z = 0.;
        double i_dev_angle = 0.;

        for(int i = 0 ; i<deviations_array_.size() ;i++)
        {
            i_dev_x += deviations_array_[i].x;
            i_dev_y += deviations_array_[i].y;
            i_dev_angle = deviations_array_[i].angle;
        }

        i_dev_x = i_dev_x/deviations_array_.size();
        i_dev_y = i_dev_y/deviations_array_.size();
        i_dev_angle = i_dev_angle/deviations_array_.size();
         
        ardrone_vel_.linear.x = 0.;
        ardrone_vel_.linear.y = i_gain[1] *i_dev_x + p_gain[1] * deviations_array_[deviations_array_.size() - 1].x;
        ardrone_vel_.linear.z = i_gain[2] *i_dev_y + p_gain[2] * deviations_array_[deviations_array_.size() - 1].y;
        ardrone_vel_.angular.z = i_dev_angle* i_angle_gain + p_angle_gain * deviations_array_[deviations_array_.size() - 1].angle;

    }

    //average_deviation_x_image_ = (average_deviation_x_image_*(time_counter_ - 1) + target_marker_point_.x - state_marker_point_.x)/time_counter_;
    //average_deviation_y_image_ = (average_deviation_y_image_*(time_counter_ - 1) + target_marker_point_.y - state_marker_point_.y)/time_counter_;

    // ardrone_vel_.linear.x = gain[0] * pow((target_maker_area - state_marker_area_), 0.5);
    //ardrone_vel_.linear.y = gain[1] * (target_marker_point_.x - state_marker_point_.x);
    //ardrone_vel_.linear.z = gain[2] * (target_marker_point_.y - state_marker_point_.y);
    // ROS_INFO("ardrone_vel_x = %lf",ardrone_vel_.linear.x);_
    // ROS_INFO("ardrone_vel_y = %lf",ardrone_vel_.linear.y);
    // ROS_INFO("ardrone_vel_z = %lf",ardrone_vel_.linear.z);
    // vel_pub_.publish(ardrone_vel_);
}

void ArdroneFollowController::hoveringGradually()
{
    Eigen::Vector3d gain(0.00005, 0.001, 0.001);
    ardrone_vel_.linear.x = 0.;
    ardrone_vel_.linear.y = gain[1] * (average_deviation_x_image_);
    ardrone_vel_.linear.z = gain[2] * (average_deviation_y_image_);
    ardrone_vel_.angular.z = 0.;
}

int main(int argc, char** argv)
{
 	ros::init (argc, argv, "ardrone_follow_controller");
    ros::NodeHandle n;
    ros::Rate rate(10);

    ArdroneFollowController follow;
    if(!follow.time_start_flag_)
    {
        follow.pre_time_ = ros::Time::now().toSec();
        follow.time_start_flag_ = true;
    }

    ros::spin();
 	return 0;
}