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

#include <ardrone_operator/calc_area_in_contour.h>
#include <ardrone_operator/red_extraction.h>


class ArdroneFollowController
{
public:
    ArdroneFollowController();
    ~ArdroneFollowController();
    double occupied_area_ratio_red_;

private:
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void controlVelocity();

    ros::NodeHandle n_;
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher vel_pub_;
    image_transport::Publisher gray_image_pub_;
    image_transport::Subscriber image_sub_;
    
    cv::Mat ipt_image_;
    std_msgs::Empty empty_msg_;
    geometry_msgs::Twist ardrone_vel_;

    double state_marker_area_;
    Eigen::Vector2d target_marker_pos_;
    Eigen::Vector2d state_marker_pos_;

    bool takeoff_flag_;
    bool calc_finished_flag_;

    std::unique_ptr<CalcAreaInContour> marker_;
};

ArdroneFollowController::ArdroneFollowController()
{
    image_transport::ImageTransport it(n_);
    image_sub_ = it.subscribe("/ardrone/front/image_raw", 1, &ArdroneFollowController::imageCallback, this);
    // gray_image_pub_ = it.advertise("/image_contour", 10);
    takeoff_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    land_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/land",1);
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
}

ArdroneFollowController::~ArdroneFollowController()
{
}

void ArdroneFollowController::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	try
    {
        ipt_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	//error
	catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

    //赤色を抽出して白黒画像に変換
    cv::Mat red_extract_image = RedExtraction::redExtraction(ipt_image_);
    //最も面積が大きい輪郭の面積
    state_marker_area_ = CalcAreaInContour::calcAreaInContour(red_extract_image);

    if(calc_finished_flag_)
    {
        calc_finished_flag_ = false;
        state_marker_pos_ = CalcAreaInContour::calcCentroidInContour(red_extract_image);
        calc_finished_flag_ = true;
    }

    int height = ipt_image_.rows;
    int width = ipt_image_.cols;
    target_marker_pos_[0] = width / 2.;
    target_marker_pos_[1] = height / 2.;

    occupied_area_ratio_red_ = state_marker_area_ / (width * height);
    
    ROS_INFO("occupied_area_red = %lf",state_marker_area_);
    ROS_INFO("occupied_area_ratio_red = %lf",occupied_area_ratio_red_);
    ROS_INFO("x = %lf",state_marker_pos_[0]);
    ROS_INFO("y = %lf",state_marker_pos_[1]);
    ROS_INFO("height = %d",height);
    ROS_INFO("width = %d",width);

 	// sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_extract_image).toImageMsg();
 	// gray_image_pub_.publish(img_msg);

    if(occupied_area_ratio_red_ > 0.2)
    {
        takeoff_pub_.publish(empty_msg_);
        takeoff_flag_ = true;
    }
    if(occupied_area_ratio_red_ < 0.1 && takeoff_flag_ == true)
    {
        land_pub_.publish(empty_msg_);
    }
    ArdroneFollowController::controlVelocity();
}

void ArdroneFollowController::controlVelocity()
{
    Eigen::Vector3d gain(0.05, 0.05, 0.05);
    double target_maker_area;
    ardrone_vel_.linear.x += gain[0] * pow((target_maker_area - state_marker_area_), 0.5);
    ardrone_vel_.linear.y += gain[1] * (target_marker_pos_[0] - state_marker_pos_[0]);
    ardrone_vel_.linear.z += gain[2] * (target_marker_pos_[1] - state_marker_pos_[1]);
    vel_pub_.publish(ardrone_vel_);
}

int main(int argc, char** argv)
{
 	ros::init (argc, argv, "img_contour");
    ros::NodeHandle n;
    ros::Rate rate(10);

    ArdroneFollowController follow;

 	ros::spin();
 	return 0;
}