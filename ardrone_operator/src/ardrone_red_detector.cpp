#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Header.h"


cv::Mat ipt_image;


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


    int height = ipt_image.rows;
    int width = ipt_image.cols;

    cv::Mat opt_image = cv::Mat(cv::Size(width, height),CV_8UC3,cv::Scalar(0,0,0));

    for (int u = 0; u<width; u++){
        for (int v = 0; v<height;v++){

            int B = (uchar)ipt_image.at<cv::Vec3b>(v,u)[0] + 1;
            int G = (uchar)ipt_image.at<cv::Vec3b>(v,u)[1] + 1;
            int R = (uchar)ipt_image.at<cv::Vec3b>(v,u)[2] + 1; 

            double r_rate = R/(1.0*(R+G+B));

            if(r_rate>0.6){
                opt_image.at<cv::Vec3b>(v,u)[2] = 255;

            }



        }

    }



        


 	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", opt_image).toImageMsg();
 	gray_image_pub.publish(img_msg);
		
}

int main(int argc, char** argv){
 	ros::init (argc, argv, "img_red");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
        
    gray_image_pub = it.advertise("/image_nipple", 10);
	
 	ros::spin();
 	return 0;
}