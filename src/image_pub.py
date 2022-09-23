#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from picamera_hq_ros.srv import TakePicture, TakePictureResponse
import numpy as np

import time
from classes import CamHandler

cam=CamHandler(resolution=(2016, 1520), iso=100 , framerate=10, shutter_speed=40000 , wb=(2.816,1.918) , sensor_mode=0,use_video_port=False)
#cam=CamHandler(resolution=(4032, 3040), iso=100 , framerate=10, shutter_speed=50000 , wb=(2.816,1.918) , sensor_mode=0, use_video_port=False)

bridge = CvBridge()

cont=False


def start_node():
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')
    rospy.loginfo('Starting camera...')
    cam.start()
    rospy.loginfo('Camera started.')

def getFrame():
    rospy.loginfo("Taking pic...")
    img=cam.takePic()
    #print(img.shape)
    #print(img)
    #print(type(img))
    imgMsg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    return imgMsg

def pubPicture():
    imgMsg=getFrame()
    rospy.loginfo("Pic taken. Sending...")
    pub.publish(imgMsg)
    rospy.loginfo("Pic sent.")
    #rospy.Rate(1.0).sleep()  # 1 Hz

def takePicture_fcn(req):
    imgMsg=getFrame()
    return TakePictureResponse(imgMsg)

if __name__ == '__main__':
    try:
        start_node()
        if cont:
            pub = rospy.Publisher('image', Image, queue_size=10)
        else:
            take_pic_service = rospy.Service('take_picture', TakePicture, takePicture_fcn)
        while not rospy.is_shutdown():
            if cont:
                pubPicture()
            else:
                rospy.spin()
        cam.stop()
    except rospy.ROSInterruptException:
        cam.stop()
