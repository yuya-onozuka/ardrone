#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Header.h"
#include <vector>
#include <std_msgs/Empty.h>

class ArdroneFollowNavigation
{
public:
    ArdroneFollowNavigation();
    ~ArdroneFollowNavigation();
    double occupied_area_ratio_red_;

private:
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

    ros::NodeHandle n_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher gray_image_pub_;
    ros::Publisher takeoff_pub_;
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


    int height = ipt_image_.rows;
    int width = ipt_image_.cols;

    cv::Mat red_image = cv::Mat(cv::Size(width, height),CV_8UC1);

    double r_rate = 0.;

    for (int u = 0; u<width; u++){
        for (int v = 0; v<height;v++)
        {

            int B = (uchar)ipt_image_.at<cv::Vec3b>(v,u)[0] + 1;
            int G = (uchar)ipt_image_.at<cv::Vec3b>(v,u)[1] + 1;
            int R = (uchar)ipt_image_.at<cv::Vec3b>(v,u)[2] + 1; 

            r_rate = R/(1.0*(R+G+B));
 
            if(r_rate>0.5)
            {
                red_image.at<unsigned char>(v,u) = 255;
                // cv::cvtColor(ipt_image,red_image,CV_BGR2GRAY);
            }
            else
            { 
                red_image.at<unsigned char>(v,u) = 0;
            }
        }
    }

    std::vector<std::vector<cv::Point>> contours;
    findContours(red_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    double max_area_red=0;
    int max_area_contour_red=-1;
    for(int j=0;j<contours.size();j++)
    {
        double occupied_area_red=contourArea(contours.at(j));
        if(max_area_red<occupied_area_red)
        {
            max_area_red=occupied_area_red;
            max_area_contour_red=j;
        }
    }    

    occupied_area_ratio_red_ = max_area_red / (height*width);
    std::cout << occupied_area_ratio_red_ << std::endl;
    ROS_INFO("height = %d",height);
    ROS_INFO("width = %d",width);
    ROS_INFO("occupied_area_red = %lf",max_area_red);
    ROS_INFO("occupied_area_ratio_red = %lf",occupied_area_ratio_red_);

 	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_image).toImageMsg();
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