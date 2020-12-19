#!/usr/bin/env python

import io
import cv2
import rospy
import picamera
import picamera.array
import time
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    with picamera.PiCamera(resolution=(640, 480)) as camera:
        camera.start_preview()
        time.sleep(2)
        camera.stop_preview()

        rate = rospy.Rate(5)        
        while True:
            with picamera.array.PiRGBArray(camera) as stream:
                camera.capture(stream, format='bgr')

                image = stream.array

                image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
                pub.publish(image_message)
                print("Pubbed")
                
                rate.sleep()


if __name__=="__main__":
    bridge = CvBridge()

    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('image_raw', Image, queue_size=10)
    
    main()
    
    cv2.destroyAllWindows()
            