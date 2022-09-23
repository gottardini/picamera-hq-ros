import rospy
from picamera import PiCamera
from picamera import mmal, mmalobj, exc
from picamera.mmalobj import to_rational
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
        
        self.MMAL_PARAMETER_ANALOG_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x59
        self.MMAL_PARAMETER_DIGITAL_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x5A

    def set_gain(self, gain, value):
        """Set the analog gain of a PiCamera.
    
        camera: the picamera.PiCamera() instance you are configuring
        gain: either MMAL_PARAMETER_ANALOG_GAIN or MMAL_PARAMETER_DIGITAL_GAIN
        value: a numeric value that can be converted to a rational number.
        """
        if gain not in [self.MMAL_PARAMETER_ANALOG_GAIN, self.MMAL_PARAMETER_DIGITAL_GAIN]:
            raise ValueError("The gain parameter was not valid")
        ret = mmal.mmal_port_parameter_set_rational(self.camera._camera.control._port, gain, to_rational(value))
        if ret == 4:
            raise exc.PiCameraMMALError(ret, "Are you running the latest version of the userland libraries? Gain setting was introduced in late 2017.")
        elif ret != 0:
            raise exc.PiCameraMMALError(ret)
            
    def set_analog_gain(self, value):
        """Set the gain of a PiCamera object to a given value."""
        self.set_gain(self.MMAL_PARAMETER_ANALOG_GAIN, value)

    def set_digital_gain(self, value):
        """Set the digital gain of a PiCamera object to a given value."""
        self.set_gain(self.MMAL_PARAMETER_DIGITAL_GAIN, value)
        
     
    def printGains(self):
        print("Mode:",self.camera.exposure_mode,"Analog:",self.camera.analog_gain, "Digital:", self.camera.digital_gain, "ISO:", self.camera.iso, "shutter:", self.camera.shutter_speed, "Exposure speed:",self.camera.exposure_speed,"WB:",self.camera.awb_gains)  
      
    def start(self):
        self.camera = PiCamera(resolution=self.resolution, framerate=None,  sensor_mode=self.sensor_mode)
        self.camera.exposure_mode="auto"
        self.set_analog_gain(1)
        self.set_digital_gain(2)
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = self.wb
        self.camera.shutter_speed = self.shutter_speed
        #self.camera.iso = self.iso
        self.printGains()
        time.sleep(5)
        #self.camera.exposure_mode="off"
        self.printGains()

        
        
    def stop(self):
        print("Closing camera...")
        self.camera.close()

    def takePic(self):
        self.printGains()
        image = np.empty((self.resolution[1] * self.resolution[0] * 3,), dtype=np.uint8)
        self.camera.capture(image, format='bgr', use_video_port=self.use_video_port)
        image = image.reshape((self.resolution[1], self.resolution[0], 3))
        #print(type(image))
        return image
