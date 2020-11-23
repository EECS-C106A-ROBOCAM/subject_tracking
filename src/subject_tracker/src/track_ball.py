#!/usr/bin/env python


import cv2
import rospy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class BallTracker:

    TARGET_RADIUS = 5
    FOCAL_LENGTH = 1

    def __init__(self):
        self.subscriber = rospy.Subscriber("input/image_raw", Image, self.locate_ball, queue_size=1)

        self.publisher = rospy.Publisher("output/adjustment_pose", Pose, queue_size=1)

        self.bridge = CvBridge()


    def locate_ball(self, msg):
        # print("callback")
        image = self.bridge.imgmsg_to_cv2(msg, "mono8")

        blurred = cv2.medianBlur(image, 5)

        rows = blurred.shape[0]
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                param1=100, param2=30,
                                minRadius=1, maxRadius=30)
        # TODO(JS): tune the bounds on the Hough parameters
        
        if circles is not None:
            circles = np.uint16(np.around(circles))

            target = circles[0]
            center = (target[0], target[1])
            radius = target[2]

            x_adj = center[0] - image.rows()
            y_adj = center[1] - image.cols()
            z_adj = (radius - BallTracker.TARGET_RADIUS) / BallTracker.FOCAL_LENGTH

            cv2.circle(image, center, 1, (100, 0, 100), 3)
            cv2.circle(image, center, radius, (0, 255, 255), 3)

            # Also draw all circles
            for i in circles[1, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(image, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(image, center, radius, (255, 0, 255), 3)
        else:
            print("no circles found D;")
    
    
        cv2.imshow("detected circles", image)
        cv2.waitKey(1)




def main():
    rospy.init_node("ball_tracker")
    tracker = BallTracker()

    rospy.spin()

if __name__ == "__main__":
    main()