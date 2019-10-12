#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from keyboard.msg import Key

UP = 273
DOWN = 274
RIGHT = 275
LEFT = 276
ENTER = 13
SPACE = 32
BACKSPACE = 8
KEY_w = 119
KEY_s = 115
KEY_a = 97
KEY_d = 100

VELOCITY_INCLIMENT = 0.5

class KeyboardOperation:

    empty_msg = Empty()
    ardrone_vel = Twist()
    ardrone_vel.linear.x = 0
    ardrone_vel.linear.y = 0
    ardrone_vel.linear.z = 0
    ardrone_vel.angular.z = 0

    def __init__(self):
        rospy.init_node('ardrone_keyboard_operation')
        self.pub_takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('ardrone/reset', Empty, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def subscribe_key_data(self):
        rospy.Subscriber("keyboard/keydown", Key, self.keydownCallback)
        rospy.Subscriber("keyboard/keyup", Key, self.keyupCallback)

    def keydownCallback(self, key):
        if key.code == ENTER:
            self.pub_takeoff.publish(self.empty_msg)
            rospy.loginfo("takefoff")
        elif key.code == SPACE:
            self.pub_land.publish(self.empty_msg)
            rospy.loginfo("land")
        elif key.code == BACKSPACE:
            self.pub_reset.publish(self.empty_msg)
            rospy.loginfo("reset")

        elif key.code == UP:
            self.ardrone_vel.linear.z += VELOCITY_INCLIMENT
        elif key.code == DOWN:
            self.ardrone_vel.linear.z -= VELOCITY_INCLIMENT
        elif key.code == LEFT:
			self.ardrone_vel.angular.z += VELOCITY_INCLIMENT
        elif key.code == RIGHT:
			self.ardrone_vel.angular.z -= VELOCITY_INCLIMENT

        elif key.code == KEY_w:
            self.ardrone_vel.linear.x += VELOCITY_INCLIMENT
        elif key.code == KEY_s:
            self.ardrone_vel.linear.x -= VELOCITY_INCLIMENT

        elif key.code == KEY_a:
            self.ardrone_vel.linear.y += VELOCITY_INCLIMENT
        elif key.code == KEY_d:
            self.ardrone_vel.linear.y -= VELOCITY_INCLIMENT
    
    def keyupCallback(self, key):
        pass
    
    def publish_vel(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_vel.publish(self.ardrone_vel)
            rate.sleep()
    

if __name__ == '__main__':
    operator = KeyboardOperation()
    operator.subscribe_key_data()
    operator.publish_vel()

