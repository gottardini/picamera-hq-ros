import rospy
from picamera import PiCamera
import time
import numpy as np

class CamHandler:
    def __init__(self, resolution, iso, framerate, shutter_speed, wb, sensor_mode, use_video_port):
        self.resolution=resolution
        self.iso=iso
        self.framerate=framerate
        self.shutter_speed=shutter_speed
        self.wb=wb
        self.sensor_mode=sensor_mode
        self.use_video_port=use_video_port
        self.camera=None
        
    def start(self):
        self.camera = PiCamera(resolution=self.resolution, framerate=self.framerate,  sensor_mode=self.sensor_mode)

        self.camera.iso = self.iso
        #time.sleep(5)
        #print(self.camera.awb_gains)
        self.camera.shutter_speed = self.shutter_speed
        self.camera.exposure_mode = 'off'
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = self.wb
        
    def stop(self):
        print("Closing camera...")
        self.camera.close()

    def takePic(self):
        image = np.empty((self.resolution[1] * self.resolution[0] * 3,), dtype=np.uint8)
        self.camera.capture(image, format='bgr', use_video_port=self.use_video_port)
        image = image.reshape((self.resolution[1], self.resolution[0], 3))
        print(type(image))
        return image
