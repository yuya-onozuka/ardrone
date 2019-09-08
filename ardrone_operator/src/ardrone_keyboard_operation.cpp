#include "ros/ros.h"
#include <keyboard/Key.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

enum Key
{
    UP = 273,
    DOWN = 274,
    RIGHT = 275,
    LEFT = 276,
    ENTER = 13,
    SPACE = 32,
    BACKSPACE = 8,
    KEY_w = 119,
    KEY_s = 115,
    KEY_a = 97,
    KEY_d = 100,
};

class KeyboardOperation
{
public:
    KeyboardOperation();
    ~KeyboardOperation();
private:
    void callbackKeyDown(const keyboard::KeyConstPtr &key_msg);
    void callbackKeyUp(const keyboard::KeyConstPtr &key_msg);

    ros::NodeHandle n_;
    ros::Subscriber sub_keydown_, sub_keyup_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_takeoff_;
    ros::Publisher pub_land_;
    ros::Publisher pub_reset_;

    geometry_msgs::Twist vel_;
    std_msgs::Empty empty_msg_;
}

KeyboardOperation::KeyboardOperation()
{
    sub_keydown_ = n_.subscribe("/keyboard/keydown",10);
    sub_keyup_ = n_.subscribe("/keyboard/keyup",10);
    pub_takeoff_ = n_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    pub_land_ = n_.advertise<std_msgs::Empty>("/ardrone/land",1);
    pub_reset_ = n_.advertise<std_msgs::Empty>("/ardrone/reset",1);
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

KeyboardOperation::~KeyboardOperation()
{
}

KeyboardOperation::callbackKeyDown(const keyboard::KeyConstPtr &key_msg)
{
    int key_code = key_msg->code;
    switch (key_code)
    {
        case Key::Enter:
            pub_takeoff_.publish(empty_msg_);
            break;
        case Key::SPACE:
            pub_land_.publish(empty_msg_);
            break;
        case Key::BACKSPACE:
            pub_reset_.publish(empty_msg_);
        case Key::UP:
            vel_.linear.z += 1.0;
            break;
        case Key::DOWN:
            vel_.linear.z -= 1.0;
            break;
        case Key::LEFT:
            vel_.angular.z += 1.0;
            break;
        case Key::RIGHT:
            vel_.angular.z -= 1.0;
            break;
        case Key::KEY_w:
            vel_.linear.x += 1.0;
            break;
        case Key::KEY_s:
            vel_.linear.x -= 1.0;
            break;
        case Key::KEY_a:
            vel_.linear.y += 1.0;
            break;
        case Key::KEY_d:
            vel_.linear.y -= 1.0;
            break;
    }
    pub_vel_.publish(vel_);
}

KeyboardOperation::callbackKeyUp(const keyboard::KeyConstPtr &key_msg)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_keyboard_operation");
    ros::Rate loop_rate(10);

    KeyboardOperation key;

    ros::spin();
    return 0;
}