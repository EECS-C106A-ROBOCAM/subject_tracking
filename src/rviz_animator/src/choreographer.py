#!/usr/bin/env python

import sys
import time
import math
import PyKDL
import rospy
import rospkg
import tf2_ros
import threading
import warnings
import traceback
import numpy as np
import kdl_parser_py.urdf as parser
from urdf_parser_py.urdf import URDF

warnings.filterwarnings('ignore', category=UserWarning)
import quaternion

from tf_conversions import posemath
from tf.transformations import quaternion_from_euler, quaternion_multiply
from scipy.interpolate import interp1d

from rviz_animator.msg import KeyframesMsg
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import (    
	Point,
	Quaternion,
    Pose,
	PoseArray,
    PoseStamped,
    Transform
)
from sensor_msgs.msg import (
    JointState
)
from std_msgs.msg import (
    Float32MultiArray
)

from ik_solver.srv import (
    SolveIKSrv
)

def createSequence(keyframes, dt=0.1):
    timestamps = [frame.timestamp for frame in keyframes]
    rotations = quaternion.as_quat_array([[frame.frame.orientation.x, frame.frame.orientation.y, frame.frame.orientation.z, frame.frame.orientation.w] for frame in keyframes])

    # Coordinate frame conversion
    multiplier_quat = quaternion.from_float_array([np.sqrt(2) / 2, -np.sqrt(2) / 2, 0, 0])
    rotations = [rot * multiplier_quat for rot in rotations]

    positions = np.array([[frame.frame.position.x, frame.frame.position.y, frame.frame.position.z] for frame in keyframes])

    time_interps = list(np.arange(timestamps[0], timestamps[-1], dt)) + [timestamps[-1]]
    lerp = interp1d(timestamps, positions, axis=0) 
    lin_interps = lerp(time_interps)

    rot_interps = []
    for i in range(len(keyframes) - 1):
        rot_interps.extend(quaternion.as_float_array(quaternion.slerp(rotations[i], rotations[i + 1], timestamps[i], timestamps[i + 1], np.arange(timestamps[i], timestamps[i + 1], dt))))
    rot_interps.append(quaternion.as_float_array(rotations[-1]))

    sequence = []
    for pos, ori in zip(lin_interps, rot_interps):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = ori
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        sequence.append(pose_stamped)


    g_poses[0] = sequence[-1].pose
    g_poses[1] = sequence[-1].pose

    return time_interps, sequence 

def callback(message):
    global g_joint_angles

    time_interps, sequence = createSequence(message.keyframes)    

    print("Getting reference frames...")
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    transform_object_to_base = None
    while not transform_object_to_base:
        try:
            transform_object_to_base = tf_buffer.lookup_transform('base', 'tracked_object', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
    
    print("Beginning playback...")
    start = time.time()
    for t, pose in zip(time_interps, sequence):
        while time.time() - start < t:
            continue
        
        try:
            transform_object_to_base = tf_buffer.lookup_transform('base', 'tracked_object', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Failed to get object to base transform at timestep {} | Real Time: {}".format(t, time.time() - start))
            continue

        target_pose = do_transform_pose(pose, transform_object_to_base)
        # quat_rot = quaternion_from_euler(0, 0, math.pi/2)
        # target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w = quaternion_multiply(quat_rot, [target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
        try:
            solve_ik = rospy.ServiceProxy('solve_ik', SolveIKSrv)
            # print(target_pose)
            response = solve_ik(target_pose)
            if response.solved:
                g_joint_angles = response.output_joints.position
                print("Setting joint state: {}".format(g_joint_angles))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            
    print("Finished playback.")

def joint_publisher():
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        publish_joints = JointState()
        publish_joints.name = ['joint_base_rot', 'joint_rot_1', 'joint_f1_2', 'joint_f2_pitch', 'joint_pitch_yaw', 'joint_yaw_roll']
        publish_joints.position = g_joint_angles
        publish_joints.header.stamp = rospy.Time.now()
        pub_joints.publish(publish_joints)

        poseArray = PoseArray()
        poseArray.header.stamp = rospy.Time.now()
        poseArray.header.frame_id = "world"
        poseArray.poses = g_poses
        pub_rviz.publish(poseArray)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('choreographer', anonymous=True)
    pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)
    g_joint_angles = [0] * 6

    pub_rviz = rospy.Publisher('target_poses', PoseArray, queue_size=10)
    g_poses = [Pose(), Pose()] 

    joint_pub_thread = threading.Thread(target=joint_publisher)
    joint_pub_thread.start()

    print("Waiting for IK service...")
    rospy.wait_for_service('solve_ik')

    print("Ready for keyframes!")
    rospy.Subscriber("operator_keyframes", KeyframesMsg, callback)

    rospy.spin()