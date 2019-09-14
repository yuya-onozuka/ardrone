#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Header.h"
#include <vector>
#include <std_msgs/Empty.h>
#include <ardrone_operator/contour_area_calc.h>
#include <ardrone_operator/red_extraction.h>

class ArdroneFollowNavigation
{
public:
    ArdroneFollowNavigation();
    ~ArdroneFollowNavigation();
    double occupied_area_ratio_red_;

private:
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

    ros::NodeHandle n_;
    ros::Publisher takeoff_pub_;
    image_transport::Publisher gray_image_pub_;
    image_transport::Subscriber image_sub_;
    
    cv::Mat ipt_image_;
    std_msgs::Empty empty_msg_;
};

ArdroneFollowNavigation::ArdroneFollowNavigation()
{
    image_transport::ImageTransport it(n_);
    image_sub_ = it.subscribe("/ardrone/front/image_raw", 1, &ArdroneFollowNavigation::imageCallback, this);
    gray_image_pub_ = it.advertise("/image_contour", 10);
    takeoff_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
}

ArdroneFollowNavigation::~ArdroneFollowNavigation()
{
}

void ArdroneFollowNavigation::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
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

    cv::Mat red_extract_image = RedExtraction::redExtraction(ipt_image_);
    double max_area_red = ContourAreaCalc::contourAreaCalc(red_extract_image);

    occupied_area_ratio_red_ = max_area_red / (ipt_image_.rows*ipt_image_.cols);
    std::cout << occupied_area_ratio_red_ << std::endl;
    
    ROS_INFO("occupied_area_red = %lf",max_area_red);
    ROS_INFO("occupied_area_ratio_red = %lf",occupied_area_ratio_red_);

 	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_extract_image).toImageMsg();
 	gray_image_pub_.publish(img_msg);

    if(max_area_red > 0.2)
    {
        // takeoff_pub_.publish(empty_msg_);
    }
}

int main(int argc, char** argv)
{
 	ros::init (argc, argv, "img_contour");
    ros::NodeHandle n;
    ros::Rate rate(10);

    ArdroneFollowNavigation follow;

 	ros::spin();
 	return 0;
}