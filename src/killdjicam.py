#!/usr/bin/python

import rospy

def camera_infocallback(data):
    print data
    

if __name__ == '__main__':

    rospy.init_node('killdjicam', anonymous=False)

    rospy.Subscriber("dji_sdk/camera_info", CameraInfo, camera_infocallback)

    rospy.spin()
