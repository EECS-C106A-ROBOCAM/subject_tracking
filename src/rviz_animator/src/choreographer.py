#!/usr/bin/env python

import sys
import time
import math
import PyKDL
import rospy
import rospkg
import tf2_ros
import warnings
import traceback
import numpy as np
import kdl_parser_py.urdf as parser

warnings.filterwarnings('ignore', category=UserWarning)
import quaternion

from tf_conversions import posemath
from urdf_parser_py.urdf import URDF
from scipy.interpolate import interp1d

from rviz_animator.msg import KeyframesMsg
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import (    
	Point,
	Quaternion,
	Pose,
    PoseStamped,
    Transform
)
from sensor_msgs.msg import (
    JointState
)

def createSequence(keyframes, dt=0.1):
    timestamps = [frame.timestamp for frame in keyframes]
    rotations = quaternion.as_quat_array([[frame.frame.orientation.x, frame.frame.orientation.y, frame.frame.orientation.z, frame.frame.orientation.w] for frame in keyframes])
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
    return time_interps, sequence 

def callback(message):
    time_interps, sequence = createSequence(message.keyframes)


    print("Setting up kinematics solvers..")
    rospkg.RosPack().get_path('rviz_animator')
    ok, tree = parser.treeFromFile(rospkg.RosPack().get_path('rviz_animator') + "/models/robocam.xml")
    chain = tree.getChain("base", "link_roll")
    current_joints = PyKDL.JntArray(chain.getNrOfJoints())
    PyKDL.SetToZero(current_joints)

    robot = URDF.from_parameter_server()
    min_joints, max_joints = PyKDL.JntArray(chain.getNrOfJoints()), PyKDL.JntArray(chain.getNrOfJoints())
    i = 0
    for joint in robot.joints:
        if joint.limit:
            min_joints[i] = joint.limit.lower
            max_joints[i] = joint.limit.upper
            i += 1
    print(min_joints)

    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(chain)
    ik_solver = PyKDL.ChainIkSolverPos_NR(chain, fk_solver, ik_vel_solver)

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
        target_frame = posemath.fromMsg(target_pose.pose)

        target_joints = PyKDL.JntArray(chain.getNrOfJoints())
        success = ik_solver.CartToJnt(current_joints, target_frame, target_joints)

        if success != 0:
            print("IK Solver Failed. Status: {}".format(success))
 
        target_state = JointState()
        target_state.header.stamp = rospy.Time.now()
        target_state.position = list(target_joints)
        target_state.name = ['joint_base_rot', 'joint_rot_1', 'joint_f1_2', 'joint_f2_pitch', 'joint_pitch_yaw', 'joint_yaw_roll']
        
        for i in range(len(target_state.position)):
            if target_state.position[i] < min_joints[i] or target_state.position[i] > max_joints[i]:
                print("Target joint {} out of bounds. Min: {}. Max: {}. Target {}.".format(target_state.name[i], min_joints[i], max_joints[i], target_state.position[i]))
            else:
                print("good")

        pub_robot.publish(target_state)
        pub.publish(target_pose)

    print("Finished playback.")

if __name__ == '__main__':
    rospy.init_node('choreographer', anonymous=True)
    pub = rospy.Publisher('rviz_poses', PoseStamped, queue_size=10)
    pub_robot = rospy.Publisher('joint_states', JointState, queue_size=10)

    print("Listening for keyframes...")
    rospy.Subscriber("operator_keyframes", KeyframesMsg, callback)

    rospy.spin()