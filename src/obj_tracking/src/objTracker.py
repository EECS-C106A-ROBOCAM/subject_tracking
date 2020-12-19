#!/usr/bin/env python

import tf 
import rospy
import pickle
import tf2_ros
import cv2, PIL
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
    with open('./src/aruco_pkg/data/calib_mtx_dist.pkl', 'rb') as handle:
        mtx, dist = pickle.load(handle)

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')

    h, w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    frame = dst[y:y+h, x:x+w]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, borderColor=(0, 0, 255))#ids)

    if ids is not None and len(ids) > 0:
        # Estimate the posture per each Aruco marker
        rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corners, 1, mtx, dist)
        
        frame_markers = aruco.drawAxis(frame_markers, mtx, dist, rotation_vectors[0], translation_vectors[0], 1)

        # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
        rotation_matrix = np.array([[0, 0, 0, 0],
                                    [0, 0, 0, 0],
                                    [0, 0, 0, 0],
                                    [0, 0, 0, 1]],
                                    dtype=float)
        rotation_matrix[:3, :3], _ = cv2.Rodrigues(rotation_vectors[0])

        # convert the matrix to a quaternion
        quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

        #broadcast frame
        tf2Stamp = TransformStamped()
        tf2Stamp.header.stamp = rospy.Time.now()
        tf2Stamp.header.frame_id = 'link_camera'
        tf2Stamp.child_frame_id = 'tracked_object'
        tf2Stamp.transform.translation.x, tf2Stamp.transform.translation.y, tf2Stamp.transform.translation.z = translation_vectors[0][0]
        tf2Stamp.transform.rotation.x, tf2Stamp.transform.rotation.y, tf2Stamp.transform.rotation.z, tf2Stamp.transform.rotation.w = quaternion
        tf2Broadcast.sendTransform(tf2Stamp)

if __name__ == '__main__':
    tf2Broadcast = tf2_ros.TransformBroadcaster()
    rospy.init_node("obj_tracker")
    sub = rospy.Subscriber("subject_image_raw", Image, track_aruco)
    rospy.spin()