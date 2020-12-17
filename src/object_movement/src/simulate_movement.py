#!/usr/bin/env python

import math
import rospy
import rospkg
import tf2_ros
import numpy as np

from geometry_msgs.msg import (
    Pose,
    Point,
    Twist,
    TransformStamped
)
from tf.transformations import quaternion_from_euler, quaternion_multiply

def register_frame(pose):
    tf2Stamp = TransformStamped()
    tf2Stamp.header.stamp = rospy.Time.now()
    tf2Stamp.header.frame_id = 'world'
    tf2Stamp.child_frame_id = 'tracked_object'
    tf2Stamp.transform.translation.x, tf2Stamp.transform.translation.y, tf2Stamp.transform.translation.z = pose.position.x, pose.position.y, pose.position.z
    tf2Stamp.transform.rotation.x, tf2Stamp.transform.rotation.y, tf2Stamp.transform.rotation.z, tf2Stamp.transform.rotation.w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    tf2Broadcast.sendTransform(tf2Stamp)

def simulate_movement(reference_frame="world", max_speed=1, refresh_rate=30, update_rate=0.5, x_min=-0.1, y_min=-0.1, x_max=0.1, y_max=0.1):
    rate = rospy.Rate(refresh_rate)
    try:
        curr_pose, speed, theta = Pose(), 0, 0
        curr_pose.orientation.w = 1
        i = 0
        while not rospy.is_shutdown():
            if i % (update_rate * refresh_rate) == 0:
                speed, theta = np.random.uniform(0, max_speed), np.random.uniform(0, 2 * math.pi)
                curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w = quaternion_from_euler(0, 0, theta)
                
            curr_pose.position.x = max(x_min, min(x_max, curr_pose.position.x + np.cos(theta) * speed / refresh_rate))
            curr_pose.position.y = max(y_min, min(y_max, curr_pose.position.y + np.sin(theta) * speed / refresh_rate))

            register_frame(curr_pose)
            i += 1
            rate.sleep()

    except rospy.ROSInterruptException, e:
        print("ROS interrupt: {0}".format(e))


def simulate_movement_linear(reference_frame="world", max_speed=1, refresh_rate=30, speed=0.1, limit=0.25):
    rate = rospy.Rate(refresh_rate)

    try:
        curr_pose= Pose()
        curr_pose.orientation.w = 1
        curr_pose.position.x = -limit
        while not rospy.is_shutdown():      
            while curr_pose.position.x < limit:          
                curr_pose.position.x += speed / refresh_rate
                register_frame(curr_pose)
                rate.sleep()
            while curr_pose.position.x > -limit:          
                curr_pose.position.x -= speed / refresh_rate           
                register_frame(curr_pose)
                rate.sleep()
            curr_pose.position.x = -limit
            
    except rospy.ROSInterruptException, e:
        print("ROS interrupt: {0}".format(e))

def main():
    rospy.init_node("movement_simulator")

    print("Running simulation")
    simulate_movement_linear()

if __name__ == '__main__':
    tf2Broadcast = tf2_ros.TransformBroadcaster()
    main()
