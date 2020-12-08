#!/usr/bin/env python

import rospy
import sys
import rospy
import rospkg
import tf2_ros
import traceback
import numpy as np
import kdl_parser_py.urdf as parser
import PyKDL
# import quaternion
import time


from scipy.interpolate import interp1d
from tf2_geometry_msgs import do_transform_pose
from rviz_animator.msg import KeyframesMsg
from geometry_msgs.msg import (    
	Point,
	Quaternion,
	Pose,
    PoseStamped,
    Transform
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

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    print("Acquiring reference frames...")
    transform_object_to_base = None
    while not transform_object_to_base:
        try:
            transform_object_to_base = tf_buffer.lookup_transform('world', 'tracked_object', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
    
    print("Beginning playback...")
    start = time.time()
    for t, pose in zip(time_interps, sequence):
        while time.time() - start < t:
            continue
        
        try:
            transform_object_to_base = tf_buffer.lookup_transform('world', 'tracked_object', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Failed to get object to base transform at timestep {} | Real Time: {}".format(t, time.time() - start))
            continue

        target_pose = do_transform_pose(pose, transform_object_to_base)
        pub.publish(target_pose)
        # do ik and get joint angles

        # send to arduino
    print("Finished playback...")

def listener():
    print("Listening for keyframes...")
    rospy.Subscriber("operator_keyframes", KeyframesMsg, callback)

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('choreographer', anonymous=True)
    pub = rospy.Publisher('rviz_poses', PoseStamped, queue_size=10)

    rospkg.RosPack().get_path('rviz_animator')
    ok, tree = parser.treeFromFile(rospkg.RosPack().get_path('rviz_animator') + "/models/robocam.urdf")
    chain = tree.getChain("base", "link_roll")
    current_joints = PyKDL.JntArray(chain.getNrOfJoints())
    PyKDL.SetToZero(current_joints)
    
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(chain)
    ik_solver = PyKDL.ChainIkSolverPos_NR(chain, fk_solver, ik_vel_solver)

    print("Solvers ready")

    # output_frame = PyKDL.Frame()
    # success = fk_solver.JntToCart(current_joints, output_frame)
    # # print("FK Solver Status: {}".format(success))
    # # print(output_frame)

    # velocity_twist = PyKDL.Twist(PyKDL.Vector(1, 1, 1), PyKDL.Vector())
    # output_joints = PyKDL.JntArray(chain.getNrOfJoints())
    # success = ik_vel_solver.CartToJnt(current_joints, velocity_twist, output_joints)
    # print("IK Vel Solver Status: {}".format(success))
    # print(output_joints)


    # Modify this for testing various frames
    goal_frame = PyKDL.Frame(PyKDL.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), PyKDL.Vector(3, 1, 1))


    goal_joints = PyKDL.JntArray(chain.getNrOfJoints())
    success = ik_solver.CartToJnt(current_joints, goal_frame, goal_joints)
    print("IK Solver Status: {}".format(success))
    # print(goal_joints)

    print("Test: does FK get us the goal pose back?")
    output_frame = PyKDL.Frame()
    success = fk_solver.JntToCart(goal_joints, output_frame)
    print("Requested frame:\n{}".format(goal_frame))
    print("Returned frame:\n{}".format(output_frame))


    listener()
