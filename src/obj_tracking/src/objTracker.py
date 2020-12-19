#!/usr/bin/env python

import tf 
import rospy
import pickle
import tf2_ros
import cv2, PIL
import rospkg
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from cv2 import aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    TransformStamped
)
from sensor_msgs.msg import (
    Image
)

def track_aruco(message):
    with open(rospkg.RosPack().get_path('obj_tracking') + '/data/calib_mtx_dist.pkl', 'rb') as handle:
        mtx, newcameramtx, dist, rvecs, tvecs = pickle.load(handle)

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')


    h, w = img.shape[:2]

    # undistort
    frame = cv2.undistort(img, mtx, dist, None, newcameramtx)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, borderColor=(0, 0, 255))#ids)

    if ids is not None and len(ids) > 0:
        # Estimate the posture per each Aruco marker
        rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corners, 1, mtx, dist)
        
        for rvec, tvec in zip(rotation_vectors, translation_vectors):
            frame_markers = aruco.drawAxis(frame_markers, mtx, dist, rvec, tvec, 1)
            x = tvec[0][0] * -0.01653 # Experimentally determined values
            y = tvec[0][2] * 0.01167
            z = tvec[0][1] * 0.02433
            print("x: {} y: {} z: {}".format(x, y, z))
    
        # cv2.imshow("Labelled", frame_markers)
        # cv2.waitKey(0)

        # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
        rotation_matrix = np.array([[0, 0, 0, 0],
                                    [0, 0, 0, 0],
                                    [0, 0, 0, 0],
                                    [0, 0, 0, 1]],
                                    dtype=float)
        rotation_matrix[:3, :3], _ = cv2.Rodrigues(rotation_vectors[0])

        # convert the matrix to a quaternion
        quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

        _, quat = tf_listener.lookupTransform('link_camera', 'world', rospy.Time(0))
        
        #broadcast frame
        tf2Stamp = TransformStamped()
        tf2Stamp.header.stamp = rospy.Time.now()
        tf2Stamp.header.frame_id = 'link_camera'
        tf2Stamp.child_frame_id = 'tracked_object_camera'
        tf2Stamp.transform.translation.x, tf2Stamp.transform.translation.y, tf2Stamp.transform.translation.z = x, y, z
        tf2Stamp.transform.rotation.x, tf2Stamp.transform.rotation.y, tf2Stamp.transform.rotation.z, tf2Stamp.transform.rotation.w = quat #0, 0, np.sqrt(2) / 2, np.sqrt(2) / 2
        tf2Broadcast.sendTransform(tf2Stamp)

if __name__ == '__main__':
    rospy.init_node("obj_tracker")
    sub = rospy.Subscriber("image_raw", Image, track_aruco)

    tf_listener = tf.TransformListener()
    tf2Broadcast = tf2_ros.TransformBroadcaster()

    tf_listener.waitForTransform("world", "link_camera", rospy.Time(), rospy.Duration(10.0))

    rospy.spin()