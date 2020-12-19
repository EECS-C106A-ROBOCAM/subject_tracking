import rospy
import cv2
import cv2.aruco as aruco
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class aruco_reader:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("raw_image", Image, self.aruco_check)

    def aruco_checker(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10
        corner, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


