#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <ardrone_operator/estimate_marker_pose.h>
#include <opencv2/core/eigen.hpp>

///////////カメラの特性を表すパラメータ
//カメラ内部パラメータ

// dist_coeffs << -0.519086, 0.331704, 0.013667, 0.002975, 0.000000;
// cv::Mat camera_matrix = ( cv::Mat_<double>( 3,3 ) << 553.627733, 0.000000, 317.448667, 0.000000, 550.558377, 122.189254, 0.000000, 0.000000, 1.000000);
//歪み補正
// cv::Mat distortion_coefficients =   (cv::Mat_<double>(1, 5) << -0.519086, 0.331704, 0.013667, 0.002975, 0.000000);

cv::Mat input_image;
image_transport::Publisher draw_image_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    Eigen::Matrix3d eigen_mat;
    eigen_mat << 553.627733, 0.000000, 317.448667, 
                0.000000, 550.558377, 122.189254, 
                0.000000, 0.000000, 1.000000;
    Eigen::VectorXd eigen_coeffs(5);
    eigen_coeffs << -0.519086, 0.331704, 0.013667, 0.002975, 0.000000;

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    cv::eigen2cv(eigen_mat, camera_matrix);
    cv::eigen2cv(eigen_coeffs, distortion_coefficients); 

    try
    {
        input_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	//error
	catch (cv_bridge::Exception& e) 
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
    EstimateMarkerPose emp;
    std::vector<Eigen::Matrix4d> rt_matrixes = emp.estimateMarkersPose(input_image, camera_matrix, distortion_coefficients);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", emp.draw_image_).toImageMsg();
    draw_image_pub.publish(img_msg);
}

int main(int argc, char** argv){
    ros::init (argc, argv, "estimate_marker_pose");
	ros::NodeHandle n;
    ros::Rate rate(10);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
    draw_image_pub = it.advertise("/estimate_pose_image",10);
 	
 	ros::spin();
 	return 0;
}