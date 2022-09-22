#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from picamera_hq_ros.srv import TakePicture, TakePictureResponse
import numpy as np

from picamera import PiCamera
import time
from fractions import Fraction
from classes import CamHandler

cam=CamHandler(resolution=(2016, 1520), iso=100 , framerate=10, shutter_speed=50000 , wb=(2.816,1.918) , sensor_mode=0,use_video_port=False)
#cam=CamHandler(resolution=(4032, 3040), iso=100 , framerate=10, shutter_speed=50000 , wb=(2.816,1.918) , sensor_mode=0, use_video_port=False)

bridge = CvBridge()

def start_node():
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')
    rospy.loginfo('Starting camera...')
    cam.start()
    rospy.loginfo('Camera started.')
    
def takePicture_fcn(req):
    rospy.loginfo("Taking pic...")
    img=cam.takePic()
    print(img.shape)
    #print(type(img))
    imgMsg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    rospy.loginfo("Pic taken. Sending...")
    #pub.publish(imgMsg)
    rospy.loginfo("Pic sent.")
    #rospy.Rate(1.0).sleep()  # 1 Hz    
    return TakePictureResponse(imgMsg)
 
if __name__ == '__main__':
    try:
        start_node()
        #pub = rospy.Publisher('image', Image, queue_size=10)
        take_pic_service = rospy.Service('take_picture', TakePicture, takePicture_fcn)
        while not rospy.is_shutdown():
            rospy.spin()
        cam.stop()
    except rospy.ROSInterruptException:
        cam.stop()
