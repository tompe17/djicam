#!/usr/bin/python

import rospy
import psutil
import re
import os
import signal

from sensor_msgs.msg import CameraInfo

have_camera_info = False

def camera_info_callback(data):
    global have_camera_info    
    have_camera_info = True

def kill_callback(data):
    global have_camera_info
    if have_camera_info:
        have_camera_info = False
    else:
        print "Driver seems dead, tryng to kill it so restart will work"
        procs = [p for p in psutil.process_iter() if 'joy_node' in " ".join(p.cmdline())]
        for p in procs:
            " ".join(p.cmdline())
            print "KILL:", p.pid
            os.kill(p.pid, signal.SIGKILL)
        have_camera_info = True
    

if __name__ == '__main__':

    rospy.init_node('killdcam', anonymous=False)

    rospy.Subscriber("dji_sdk/camera_info", CameraInfo, camera_info_callback)
    rospy.Timer(rospy.Duration(1), kill_callback)

    rospy.spin()
