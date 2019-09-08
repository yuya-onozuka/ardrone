#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_operator");

    ros::NodeHandle n;
   
    ros::Publisher pub_vel;
    ros::Publisher takeOff;
    ros::Publisher land;
    ros::Publisher reset;
    
    ros::Rate rate(10);
   
    geometry_msgs::Twist vel;

    std_msgs::Empty myMsg;

    takeOff = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    land = n.advertise<std_msgs::Empty>("/ardrone/land",1);
    reset = n.advertise<std_msgs::Empty>("/ardrone/reset",1);
   
    pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
   
    std::cout << "t: take off, l: land, r: reset" << std::endl;
    std::cout << "i: forward, k: backward, l: right, j: left, q: up, s: down" << std::endl;
    std::cout << "o: clockwise, u: counterclockwise" << std::endl;

    while (ros::ok()) { 
        char key;  // 入力キーの値

        std::cin >> key;
        std::cout << key << std::endl;

        switch (key) {
        case 'i': 
            vel.linear.x  =  0.1;
            break;
        case 'k':
            vel.linear.x  = -0.1;
            break;
        case 'j':
            vel.linear.y  =  0.1;
            break;
        case 'l':
            vel.linear.y  = -0.1;
            break;
        case 'q':
            vel.linear.z  =  0.1;
            break;
        case 's':
            vel.linear.z  = -0.1;
            break;        
        case 'u':
            vel.angular.z =  10.0;
            break;
        case 'o':
            vel.angular.z = -1.0;
            break;
        case 't':
            std::cout << '1' << std::endl;
            takeOff.publish(myMsg);
            break;
        case 'e':
            land.publish(myMsg);
            break;
        case 'r':
            reset.publish(myMsg);
            break;
        
        }

        pub_vel.publish(vel);    
        ros::spinOnce();  
        vel.linear.x  = 0.0; 
        vel.linear.y  = 0.0;
        vel.linear.z  = 0.0;
        vel.angular.z = 0.0; 
        rate.sleep();

    }

    return 0;
}
