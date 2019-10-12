#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_srvs.srv import Empty

def toggle_cam_client():
    rospy.wait_for_service('/ardrone/togglecam')
    try:
        service = rospy.ServiceProxy('/ardrone/togglecam', Empty)
        response = service()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":
    toggle_cam_client()