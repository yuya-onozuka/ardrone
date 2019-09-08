#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Header.h"
#include <vector>
#include <std_msgs/Empty.h>

cv::Mat ipt_image;


image_transport::Publisher gray_image_pub;
image_transport::Publisher edge_image_pub;
ros::Publisher takeOff;
ros::Publisher land;
std_msgs::Empty myMsg;
double occupied_area_ratio_red = 0;
double occupied_area_ratio_black = 0;



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

    cv::Mat red_image = cv::Mat(cv::Size(width, height),CV_8UC1);
    cv::Mat black_image = cv::Mat(cv::Size(width, height),CV_8UC1);

    double r_rate = 0.;
    double b_rate = 0.;
    for (int u = 0; u<width; u++){
        for (int v = 0; v<height;v++){

            int B = (uchar)ipt_image.at<cv::Vec3b>(v,u)[0] + 1;
            int G = (uchar)ipt_image.at<cv::Vec3b>(v,u)[1] + 1;
            int R = (uchar)ipt_image.at<cv::Vec3b>(v,u)[2] + 1; 

            r_rate = R/(1.0*(R+G+B));
            b_rate = B/(1.0*(R+G+B));

        if(r_rate>0.6){
                red_image.at<unsigned char>(v,u) = 255;
            // cv::cvtColor(ipt_image,red_image,CV_BGR2GRAY);
        }
        else if(b_rate>0.6)
        {
            black_image.at<unsigned char>(v,u) = 255;
        }
        else
        { 
            red_image.at<unsigned char>(v,u) = 0;
            black_image.at<unsigned char>(v,u) = 0;
        }
            
            
        }
    }

    
    

    std::vector<std::vector<cv::Point>> contours_red;
    std::vector<std::vector<cv::Point>> contours_black;
    findContours(red_image, contours_red, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    findContours(black_image, contours_black, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    double max_area_red=0;
    double max_area_black=0;
    int max_area_contour_red=-1;
    int max_area_contour_black=-1;
    double occupied_area_red = 0;
    double occupied_area_black = 0;
    for(int j=0;j<contours_red.size();j++)
    {
        occupied_area_red=contourArea(contours_red.at(j));
        if(max_area_red<occupied_area_red)
        {
            max_area_red=occupied_area_red;
            max_area_contour_red=j;
        }
        
    }    

    for(int j=0;j<contours_black.size();j++)
    {
        occupied_area_black=contourArea(contours_black.at(j));
        if(max_area_black<occupied_area_black)
        {
            max_area_black=occupied_area_black;
            max_area_contour_black=j;
        }
        
    }   
    occupied_area_ratio_red = occupied_area_red / (height*width);
    std::cout << occupied_area_ratio_red << std::endl;
    occupied_area_ratio_black = occupied_area_black / (height*width);
    std::cout << occupied_area_ratio_black << std::endl;


 	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_image).toImageMsg();
 	gray_image_pub.publish(img_msg);
   
		
}

int main(int argc, char** argv){
 	ros::init (argc, argv, "img_contour");
	ros::NodeHandle n;
    ros::Rate rate(10);
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
        
    gray_image_pub = it.advertise("/image_contour", 10);

    ros::Publisher takeOff = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    ros::Publisher land = n.advertise<std_msgs::Empty>("/ardrone/land",1);
    while (ros::ok()) 
    { 
        if(occupied_area_ratio_red > 0.2)
        {
            takeOff.publish(myMsg);
        }
        if(occupied_area_ratio_black > 0.2)
        {
            land.publish(myMsg);
        }
        ros::spinOnce();
        rate.sleep();
    }
	
 	ros::spin();
 	return 0;
}