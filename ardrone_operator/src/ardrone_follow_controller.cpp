#include <ardrone_operator/ardrone_follow_controller.h>
#include <ardrone_operator/project_point_to_image.h>

ArdroneFollowController::ArdroneFollowController()
{
    image_transport::ImageTransport it(n_);

    navdata_sub_ = n_.subscribe("/ardrone/navdata", 1, &ArdroneFollowController::navdataCallback, this);
    image_sub_ = it.subscribe("/ardrone/front/image_raw", 1, &ArdroneFollowController::imageCallback, this);

    draw_image_pub_ = it.advertise("/estimate_pose_image",10);
    takeoff_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    land_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/land",1);
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    is_flying_ = false;
    loadParameters();
    emp_.reset(new EstimateMarkerPose());
}

ArdroneFollowController::~ArdroneFollowController()
{
}

void ArdroneFollowController::navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& navdata_msg)
{
    ardrone_state_ = navdata_msg->state;
    //state = 0:Unknown 1:Init 2:Landed 3:Flying 4:Hovering 5:Test 6:Taking off 7:Goto Fix Point 8: Landing 9: Looping
    if(ardrone_state_ == 3 || ardrone_state_ == 4 || ardrone_state_ == 7)
    {
        is_flying_ = true;
    }
    else if(ardrone_state_ == 2 || ardrone_state_ == 8)
    {
        for(int i = 0; i < deviations_array_.size(); i++)
        {
            deviations_array_.pop_back();
        }
        is_flying_ = false;
    }
    // ROS_INFO("ardrone_state: %d", ardrone_state_);
}

void ArdroneFollowController::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    try
    {
        input_image_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
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

    rt_matrixes_ = emp_->estimateMarkersPose(input_image_, camera_matrix, distortion_coefficients, marker_size_);

    if(rt_matrixes_.size()>0)
    {
        state_marker_point_ = ProjectPointToImage::project_point_to_image(eigen_camera_matrix_, rt_matrixes_[0], marker_coordinate_target_point_);
        cv::circle(emp_->draw_image_, cv::Point(state_marker_point_.x, state_marker_point_.y), 10, cv::Scalar(0,0,255), 3, 4);
    }
    
    sensor_msgs::ImagePtr draw_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", emp_->draw_image_).toImageMsg();
    draw_image_pub_.publish(draw_img_msg);
    
    if(is_flying_==true)
    {
        computeDeviation();
        computeVelocity();
        // ROS_INFO("ardrone_vel_x = %lf",ardrone_vel_.linear.x);
        // ROS_INFO("ardrone_vel_y = %lf",ardrone_vel_.linear.y);
        // ROS_INFO("ardrone_vel_z = %lf",ardrone_vel_.linear.z);    
        vel_pub_.publish(ardrone_vel_);
    }
}

void ArdroneFollowController::computeDeviation()
{
    deviation_factors new_factor;

    if(rt_matrixes_.size() > 0)
    {
        new_factor.x = target_marker_point_.x - state_marker_point_.x;
        new_factor.y = target_marker_point_.y - state_marker_point_.y;
        new_factor.z = rt_matrixes_[0](2,3) - target_distance_;
        new_factor.angle = atan(-rt_matrixes_[0](0,3)/(rt_matrixes_[0](2,3) + 0.00001));
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

    if(deviations_array_.size() == historical_deviation_num_)
    {
        for (int i = 0; i<historical_deviation_num_ - 1; i++)
        {
            deviations_array_[i] = deviations_array_[i+1];
        }
        deviations_array_[historical_deviation_num_ - 1] = new_factor;
    }
    else
    {
         deviations_array_.push_back(new_factor);
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