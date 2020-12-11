#!/usr/bin/env python

import tf
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

# warnings.filterwarnings('ignore', category=UserWarning)
# import quaternion

from tf_conversions import posemath
from urdf_parser_py.urdf import URDF
from scipy.interpolate import interp1d
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis, quaternion_multiply, quaternion_conjugate, quaternion_from_matrix

from rviz_animator.msg import KeyframesMsg
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from geometry_msgs.msg import (    
	Point,
    PointStamped,
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
    print(chain.getNrOfJoints())
    print(robot.joints)
    for joint in robot.joints:
        if joint.limit:
            min_joints[i] = joint.limit.lower
            max_joints[i] = joint.limit.upper
            i += 1
    print(min_joints)

    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(chain)
    ik_solver = PyKDL.ChainIkSolverPos_NR_JL(chain, min_joints, max_joints, fk_solver, ik_vel_solver)

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

        # print(plan)
        # if success != 0:
        #     print("IK Solver Failed. Status: {}".format(success))
 
        # target_state = JointState()
        # target_state.header.stamp = rospy.Time.now()
        # target_state.position = list(target_joints)
        # target_state.name = ['joint_base_rot', 'joint_rot_1', 'joint_f1_2', 'joint_f2_pitch', 'joint_pitch_yaw', 'joint_yaw_roll']
        # target_state.name = ['joint_rot_1', 'joint_f1_2', 'joint_f2_pitch', 'joint_yaw_roll']
        # for i in range(len(target_state.position)):
        #     if target_state.position[i] < min_joints[i] or target_state.position[i] > max_joints[i]:
        #         print("Target joint {} out of bounds. Min: {}. Max: {}. Target {}.".format(target_state.name[i], min_joints[i], max_joints[i], target_state.position[i]))
        #     else:
        #         print("good")

        # pub_robot.publish(target_state)
        pub.publish(target_pose)

    print("Finished playback.")

def grid_search():
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

    gs_num = 10
    grid_search_angles = [(a, b, c, d, e, f) for a in np.linspace(min_joints[0], max_joints[0], gs_num) \
                                             for b in np.linspace(min_joints[1], max_joints[1], gs_num) \
                                             for c in np.linspace(min_joints[2], max_joints[2], gs_num) \
                                             for d in np.linspace(min_joints[3], max_joints[3], gs_num) \
                                             for e in np.linspace(min_joints[4], max_joints[4], gs_num) \
                                             for f in np.linspace(min_joints[5], max_joints[5], gs_num)]

    print("{} Angles computed, solving IK...".format(len(grid_search_angles)))

    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(chain)
    ik_solver = PyKDL.ChainIkSolverPos_NR_JL(chain, min_joints, max_joints, fk_solver, ik_vel_solver)    

    target_joints, target_frame = PyKDL.JntArray(chain.getNrOfJoints()), PyKDL.Frame()
    successful_joints, failed_joints = [], []
    for ags in grid_search_angles:
        for i in range(chain.getNrOfJoints()):
            target_joints[i] = ags[i]
        target_joints[4] = 0
        target_joints[5] = 0

        fk_success = fk_solver.JntToCart(target_joints, target_frame)
        success = ik_solver.CartToJnt(current_joints, target_frame, target_joints)

        if (success >= 0):
            # print("Success! Used {}".format(ags))
            successful_joints.append((ags, target_frame))
        else:
            print("Failed. Used {}".format(ags))
            failed_joints.append((ags, target_frame))
    print("Done grid searching.")

def get_cylindrical(x, y, z):
    r = np.sqrt((x ** 2) + (y ** 2))
    theta = np.arctan(y / x)
    
    return r, theta, z

if __name__ == '__main__':
    rospy.init_node('choreographer', anonymous=True)
    pub = rospy.Publisher('rviz_poses', PoseStamped, queue_size=10)
    pub_robot = rospy.Publisher('joint_states', JointState, queue_size=10)

    # initialize the joints for the bars and their limits, adding a slack joint for arbitrary orientation (because we desire a point, but IK solves a pose)
    chain = PyKDL.Chain()
    chain.addSegment(PyKDL.Segment("joint_rot_1", PyKDL.Joint(PyKDL.Joint.RotY), PyKDL.Frame(PyKDL.Vector(0, 0.01, .05))))
    chain.addSegment(PyKDL.Segment("joint_f1_2", PyKDL.Joint(PyKDL.Joint.RotY), PyKDL.Frame(PyKDL.Vector(0, 0, 0.0374))))
    chain.addSegment(PyKDL.Segment("joint_slack", PyKDL.Joint(PyKDL.Joint.RotY), PyKDL.Frame(PyKDL.Vector(0, 0.01, 0.0374))))

    current_joints = PyKDL.JntArray(chain.getNrOfJoints())
    PyKDL.SetToZero(current_joints)

    robot = URDF.from_parameter_server()
    min_joints, max_joints = PyKDL.JntArray(chain.getNrOfJoints()), PyKDL.JntArray(chain.getNrOfJoints())
    i = 0
    for joint in robot.joints:
        if joint.name in ["joint_rot_1", "joint_f1_2"]:
            min_joints[i] = joint.limit.lower
            max_joints[i] = joint.limit.upper
            i += 1
    min_joints[-1], max_joints[-1] = [-float('inf'), float('inf')]

    # initialize solvers
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(chain)
    ik_solver = PyKDL.ChainIkSolverPos_NR(chain, fk_solver, ik_vel_solver)
    
    # initialize buffer and listener for transform extraction
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # initialize publishers
    pub_pink = rospy.Publisher('pink_pose', PoseStamped, queue_size=10)
    pub_orange = rospy.Publisher('orange_point', PointStamped, queue_size=10)

    # creating pink pose with known angles (input)
    rad, deg = math.pi / 180, [10, 15, 5]
    quat_in_y = [0, 0, 0.707, 0.707]

    quat = quaternion_from_euler(deg[0] * rad, deg[1] * rad, deg[2] * rad, axes='rxzy')
    pink_quat = quaternion_multiply(quat, quat_in_y)
    pink_pos = [0.25, 0.30, 0.35]
    print(np.array(deg) * rad)
    
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pink_pos
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = pink_quat
    
    # solving 3 orientation joints from pink pose's quaternion for this test case (known in this test case, pink_quat)
    # verify that input degree angles and solved degree angles (extracted from quaternion) match
    quat_restored = quaternion_multiply(pink_quat, quaternion_conjugate(quat_in_y))
    deg_restored = np.array(euler_from_quaternion(quat_restored, axes='rxzy')) / rad
    print(deg)
    print(deg_restored)

    #TODO: THIS PART IS BAD
    # attempt at computing "orange arrow", inconclusive / unfinished / broken
    transform_cam_to_f2 = None
    while not transform_cam_to_f2:
        try:
            transform_cam_to_f2 = tf_buffer.lookup_transform('link_yaw', 'link_camera', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
    point_stamped = PointStamped()
    point_stamped.header.stamp, point_stamped.header.frame_id = rospy.Time.now(), "world"
    point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = [0, 0, 0]
    orange_pt = do_transform_point(point_stamped, transform_cam_to_f2)
    print(transform_cam_to_f2)

    r, theta, z = get_cylindrical(orange_pt.point.x, orange_pt.point.y, orange_pt.point.z)
    orange_cyl_pt = [r, theta, z]
    
    # creating desired IK pose and solving, inconclusive / unfinished / broken
    target_pose = Pose()
    target_pose.position.x, target_pose.position.z = orange_cyl_pt[0], orange_cyl_pt[2]
    target_pose.orientation.x = 0
    target_pose.orientation.y = 1
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0
    target_frame = posemath.fromMsg(target_pose)

    target_joints = PyKDL.JntArray(chain.getNrOfJoints())
    success = ik_solver.CartToJnt(current_joints, target_frame, target_joints)
    # desired is 0
    print(success)
    print(target_joints)

    ## FIN

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # br.sendTransform((orange_cyl_pt[0], 0, orange_cyl_pt[2]), (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w), rospy.Time.now(), "test", "world")
        pub_pink.publish(pose)
        pub_orange.publish(orange_pt)
        rate.sleep()

    # grid_search()
    # print("Listening for keyframes...")
    # rospy.Subscriber("operator_keyframes", KeyframesMsg, callback)

    rospy.spin()