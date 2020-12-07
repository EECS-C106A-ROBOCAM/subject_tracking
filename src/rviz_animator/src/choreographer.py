#!/usr/bin/env python

import rospy
import sys
import rospy
import tf2_ros
import traceback
import numpy as np

from tf2_geometry_msgs import do_transform_pose
from rviz_animator.msg import Keyframe
from geometry_msgs.msg import (    
	Point,
	Quaternion,
	Pose
)

from robocam_sim.src.manipulator_sim.src.path_planner import PathPlanner

def createWaypoints(origin_pose, dest_pose, step=0.25):
    intervals = np.arange(0, 1, step)
    waypoints = []

    for interval in intervals:
        waypoint = Pose()
        waypoint.orientation = origin_pose.orientation.slerp(dest_pose.orientation, interval)
        waypoint.position =  origin_pose.position.lerp(dest_pose.position, interval)

        waypoints.append(waypoint)
    waypoints.append(dest_pose)

    return waypoints

def callback(message):

    #Print the contents of the message to the console
    print("Message: %s" % (message.Frame) + ", Sent at: %s" % (message.Timestamp)
            + ", Recieved at: %s" % (rospy.get_time()))

def listener():
    rospy.Subscriber("/operator_keyframes", Keyframe, callback)

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('choreographer', anonymous=True)
    listener()
