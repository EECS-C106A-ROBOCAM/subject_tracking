#!/usr/bin/env python


import cv2
import rospy
import rospkg

import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class BallTracker:

    TARGET_RADIUS = 5
    FOCAL_LENGTH = 1

    def __init__(self):
        self.subscriber = rospy.Subscriber("input/image_raw", Image, self.locate_ball_callback, queue_size=1)

        self.publisher = rospy.Publisher("output/adjustment_pose", Pose, queue_size=1)

        self.bridge = CvBridge()


    def locate_ball_callback(self, msg):
        # print("callback")
        image = self.bridge.imgmsg_to_cv2(msg)
        locate_ball(image)

    def locate_ball(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(gray, 5)

        rows = blurred.shape[0]

        circles = cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT, 1,
                            param1=50,param2=30,minRadius=0,maxRadius=0, minDist=50)
        # TODO(JS): tune the bounds on the Hough parameters
        
        if circles is not None:
            circles = np.uint16(np.around(circles))[0]

            target = circles[0]
            print(target)
            center = (target[0], target[1])
            radius = target[2]

            x_adj = (center[0] - image.shape[0]) / BallTracker.FOCAL_LENGTH
            y_adj = (center[1] - image.shape[1]) / BallTracker.FOCAL_LENGTH
            z_adj = (radius - BallTracker.TARGET_RADIUS) / BallTracker.FOCAL_LENGTH
            print("X adj: {0}\tY adj: {1}\tZ adj: {2}".format(x_adj, y_adj, z_adj))

            cv2.circle(image, center, 1, (100, 0, 100), 3)
            cv2.circle(image, center, radius, (0, 255, 255), 3)

            # Also draw all circles
            for i in circles[1:]:
                center = (i[0], i[1])
                # # circle center
                cv2.circle(image, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(image, center, radius, (255, 0, 255), 3)
        else:
            print("no circles found D;")
    
    
        cv2.imshow("detected circles", image)
        key = cv2.waitKey(0) & 0xFF
        if key == 27:
           cv2.destroyAllWindows()




def main():
    rospy.init_node("ball_tracker")
    tracker = BallTracker()

    rospy.spin()

def test():
    tracker = BallTracker()
    images_path = rospkg.RosPack().get_path('subject_tracker')+"/images/"
    image = cv2.imread(images_path + "test0.png")
    tracker.locate_ball(image)

if __name__ == "__main__":
    # main()
    test()