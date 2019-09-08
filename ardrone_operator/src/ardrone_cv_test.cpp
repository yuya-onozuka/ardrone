#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Header.h"

cv::Mat ipt_image;
cv::Mat opt_image;
cv::Mat edge_image;

image_transport::Publisher gray_image_pub;
image_transport::Publisher edge_image_pub;


void imageCallback(const sensor_msgs::Image::ConstPtr& msg){


	try{
                ipt_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	//error
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}



                cv::cvtColor(ipt_image,opt_image,CV_BGR2GRAY);
 		sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", opt_image).toImageMsg();
 		gray_image_pub.publish(img_msg);
		cv::Canny(opt_image,edge_image,120,40,3,false);
		sensor_msgs::ImagePtr edge_msg = cv_bridge::CvImage(std_msgs::Header(),"mono8",edge_image).toImageMsg();
		edge_image_pub.publish(edge_msg);
}

int main(int argc, char** argv){
 	ros::init (argc, argv, "img_sex");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
        
        gray_image_pub = it.advertise("/image_sex", 10);
	edge_image_pub = it.advertise("/image_fuck",10);
 	ros::spin();
 	return 0;
}
