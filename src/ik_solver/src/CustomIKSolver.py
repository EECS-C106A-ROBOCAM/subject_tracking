#!/usr/bin/env python

import tf
import math
import PyKDL
import rospy
import rospkg
import numpy as np
import kdl_parser_py.urdf as parser

from urdf_parser_py.urdf import URDF

from tf_conversions import posemath
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import (
    PoseStamped,
    PointStamped
)

from sensor_msgs.msg import (
    JointState
)

from ik_solver.srv import (
    SolveIKSrv,
    SolveIKSrvResponse
)

class Plotter:
    def __init__(self):
        self.publishers = []
        self.toPublish = []
        self.labels = []
        self.goalState = JointState()

    def addGoal(self, goalState):
        self.publishers.append(rospy.Publisher("joint_states", JointState, queue_size=10))
        self.toPublish.append(goalState)
        self.labels.append("joint_states")

    def addVector(self, vector, label):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "world"
        point.point.x, point.point.y, point.point.z = vector

        self.publishers.append(rospy.Publisher("pub_{}".format(label), PointStamped, queue_size=10))
        self.toPublish.append(point)
        self.labels.append("pub_" + label)

    def publish(self):
        print("Publishing...")
        print("Labels: \n{}".format(self.labels))
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for tp, publisher in zip(self.toPublish, self.publishers):
                publisher.publish(tp)
            rate.sleep()

def callback(req):
    targetFrame = posemath.fromMsg(req.input_pose.pose) # Target pose as KDL frame
    solved, jointState = solveIK(targetFrame) 
    ret = SolveIKSrvResponse()
    ret.solved = solved
    ret.output_joints = jointState
    return ret


def solveIK(targetFrame):
    ok, tree = parser.treeFromFile(rospkg.RosPack().get_path('rviz_animator') + "/models/robocam.xml")
    chain = tree.getChain('base', 'link_camera')

    plotter = Plotter()

    # 1. Solve for J4, J5_initial, J6
    # First, convert quaternion orientation to XZY order Euler angles
    targetQuat = targetFrame.M.GetQuaternion() # Get quaternion from KDL frame (x, y, z, w)
    pitch, yaw, roll = tf.transformations.euler_from_quaternion(targetQuat, axes='rxzy')
    pitch_deg, yaw_deg, roll_deg = math.degrees(pitch), math.degrees(yaw), math.degrees(roll)

    J4_origin_orientation = posemath.toMsg(chain.getSegment(2).getFrameToTip()).orientation
    J4_offset = euler_from_quaternion([J4_origin_orientation.x, J4_origin_orientation.y, J4_origin_orientation.z, J4_origin_orientation.w])[0]

    J4_raw, J5_initial, J6 = pitch, yaw, roll
    J4 = J4_raw - J4_offset
    # 1. Complete above

    print("J4: {} J5_init: {} J6: {}".format(J4, J5_initial, J6))

    chainAngles = PyKDL.JntArray(8)
    chainAngles[5], chainAngles[6], chainAngles[7] = J4, J5_initial, J6
    chainFK = PyKDL.ChainFkSolverPos_recursive(chain)
    purpleFrame = PyKDL.Frame()
    brownFrame = PyKDL.Frame()
    
    purpleSuccess = chainFK.JntToCart(chainAngles, purpleFrame)
    # print("Purple success {}".format(purpleSuccess))

    print("Target Orientation:\n{}".format(targetFrame.M))
    print("Result Orientation:\n{}".format(purpleFrame.M))
    
    brownSuccess = chainFK.JntToCart(chainAngles, brownFrame, segmentNr=7)

    # 2. Determine position of orange point
    # First, construct KDL chain of the 3 links involved in J4-J6
    cameraOffsetChain = tree.getChain('link_pitch', 'link_camera')
    cameraJointAngles = PyKDL.JntArray(2)
    cameraJointAngles[0], cameraJointAngles[1] = J5_initial, J6
    cameraOffsetChainFK = PyKDL.ChainFkSolverPos_recursive(cameraOffsetChain)
    cameraFrame = PyKDL.Frame()
    success = cameraOffsetChainFK.JntToCart(cameraJointAngles, cameraFrame)
    # print("FK Status: {}".format(success))
    # print("Camera Frame: {}".format(cameraFrame))
    # print("End Effector Joint Angles: {}".format([J4, J5_initial, J6]))

    orangePoint = targetFrame.p - (purpleFrame.p - brownFrame.p)

    plotter.addVector(targetFrame.p, "pink")
    plotter.addVector(orangePoint, "orange")
    plotter.addVector(purpleFrame.p, "purple")
    plotter.addVector(brownFrame.p, "brown")

    # print("Target Frame Position: {}".format(targetFrame.p))
    # print("Camera Frame Position: {}".format(cameraFrame.p))
    # print("Offset: {}".format(targetFrame.p - cameraFrame.p))

    # 2. Complete:
    
    # 3. Convert orange point to cylindrical coordinates
    orange_X, orange_Y, orange_Z = orangePoint
    orange_R = math.sqrt(orange_X ** 2 + orange_Y ** 2)
    orange_Theta = math.atan2(orange_Y, orange_X) # Theta measured from global positive X axis
    
    # 3. Complete: (above)

    print("Orange R: {} Theta: {}".format(orange_R, math.degrees(orange_Theta)))

    # 4. Solve for J2 and J3 in the idealized R-Z plane
    targetVectorOrig = PyKDL.Vector(0, orange_R, orange_Z)
    plotter.addVector(targetVectorOrig, "targetRZOrig")

    # First, remove the fixed offsets from the wrist, elbow, and shoulder pieces
    wristEndFrame = PyKDL.Frame()
    wristStartFrame = PyKDL.Frame()
    elbowEndFrame = PyKDL.Frame()
    elbowStartFrame = PyKDL.Frame()
    shoulderEndFrame = PyKDL.Frame()
    shoulderStartFrame = PyKDL.Frame()

    chainFK.JntToCart(chainAngles, wristEndFrame, segmentNr=7)
    chainFK.JntToCart(chainAngles, wristStartFrame, segmentNr=5)
    chainFK.JntToCart(chainAngles, elbowEndFrame, segmentNr=4)
    chainFK.JntToCart(chainAngles, elbowStartFrame, segmentNr=3)
    chainFK.JntToCart(chainAngles, shoulderEndFrame, segmentNr=2)
    chainFK.JntToCart(chainAngles, shoulderStartFrame, segmentNr=0)

    plotter.addVector(wristEndFrame.p, "wristEndFrame")
    plotter.addVector(wristStartFrame.p, "wristStartFrame")
    plotter.addVector(elbowEndFrame.p, "elbowEndFrame")
    plotter.addVector(elbowStartFrame.p, "elbowStartFrame")
    plotter.addVector(shoulderEndFrame.p, "shoulderEndFrame")
    plotter.addVector(shoulderStartFrame.p, "shoulderStartFrame")

    wristOffset = wristEndFrame.p - wristStartFrame.p
    elbowOffset = elbowEndFrame.p - elbowStartFrame.p
    shoulderOffset = shoulderEndFrame.p - shoulderStartFrame.p
    targetVector = targetVectorOrig - wristOffset - elbowOffset - shoulderOffset

    plotter.addVector(targetVector, "targetRZ")

    # The following steps use the same labels as the classic 2D planar IK derivation
    a1, a2 = (shoulderEndFrame.p - elbowStartFrame.p).Norm(), (elbowEndFrame.p - wristStartFrame.p).Norm()
    _, ik_x, ik_y = targetVector
    
    q2_a = math.acos((ik_x ** 2 + ik_y ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2))
    q1_a = math.atan2(ik_y, ik_x) - math.atan2(a2 * math.sin(-q2_a), a1 + a2 * math.cos(-q2_a))

    q2_b = -1 * math.acos((ik_x ** 2 + ik_y ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2))
    q1_b = math.atan2(ik_y, ik_x) + math.atan2(a2 * math.sin(q2_b), a1 + a2 * math.cos(q2_b))

    # Choose 'better' set of q1_ab, q2_ab
    q1, q2 = q1_a, q2_a # TODO(JS): Is this always the better one?

    J2_initial = q1
    J2_offset = math.radians(90) # J2's zero position is vertical, not horizontal
    J2 = J2_initial - J2_offset

    # Since we have a parallel link, the angle for J3 is not simply q2. Instead, use transversal
    J3_initial = q1 - q2
    J3_offset = elbowStartFrame.M.GetRPY()[0] # J3's zero position is below horizontal
    J3 = J3_initial - J3_offset
    # 4. Complete (above)

    # print("J2: {} J3: {}".format(J2, J3))
    
    # 5. Use the Theta from cylindrical coordinates as the J1 angle, and update J5 accordingly
    J1 = orange_Theta - math.radians(90)
    J5 = J5_initial - (orange_Theta - math.radians(90))
    # 5. Complete (above)
    
    # print("J1: {} J5: {}".format(J1, J5))

    jointAngles = [J1, J2, J3, J4, J5, J6]
    jointNames = ['joint_base_rot', 'joint_rot_1', 'joint_f1_2', 'joint_f2_pitch', 'joint_pitch_yaw', 'joint_yaw_roll']

    print("\n\nFinal joint angles in radians: {}\n\n".format(jointAngles))

    in_bounds = True
    for angle, name in zip(jointAngles, jointNames):
        limit = robot_urdf.joint_map[name].limit
        if angle < limit.lower or angle > limit.upper:
            in_bounds = False
            print("Joint {} out of bounds with angle {} not between {} and {}".format(name, angle, limit.lower, limit.upper))
    print("All in bounds: {}".format(in_bounds))        

    solvedJoints = PyKDL.JntArray(8)
    solvedJoints[0], solvedJoints[1], solvedJoints[3], solvedJoints[5], solvedJoints[6], solvedJoints[7] = jointAngles
    solvedJoints[2], solvedJoints[4] = -1 * solvedJoints[1], -1 * solvedJoints[3]
    producedFrame = PyKDL.Frame()

    for i in range(chain.getNrOfSegments()):
        rc = chainFK.JntToCart(solvedJoints, producedFrame, segmentNr=i)
        plotter.addVector(producedFrame.p, "fk_produced_{}".format(i))

    # print("Result: {}".format(rc))
    # print("Output position: {}\nExpected position: {}".format(producedFrame.p, targetFrame.p))
    # print("Output orientation: {}\nExpected orientation: {}".format(producedFrame.M, targetFrame.M))


    # 6. (optional) Sanity check on solution:
    # sanityTest(BASE_TO_BASE_YAW, BASE_YAW_TO_BOTTOM_4B, targetFrame, cameraFrame, cameraOffsetChain, jointAngles)

    # 7. Create JointState message for return
    ret = JointState()
    ret.name = ['joint_base_rot', 'joint_rot_1', 'joint_f1_2', 'joint_f2_pitch', 'joint_pitch_yaw', 'joint_yaw_roll']
    ret.header.stamp = rospy.Time.now()
    ret.position = jointAngles
    
    plotter.addGoal(ret)
    # plotter.publish()

    return in_bounds, ret


def testIK():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tf_listener.waitForTransform("base", "link_camera", rospy.Time(), rospy.Duration(10.0))
        targetFrame_tf = tf_listener.lookupTransform('base', 'link_camera', rospy.Time(0))
        
        targetFrame = posemath.fromTf(targetFrame_tf)

        print("Input quaternion: {}".format(targetFrame.M.GetQuaternion()))
        print("Input vector: {}".format(targetFrame.p))
        
        jointAngles = solveIK(targetFrame)

    jointAngles = solveIK(targetFrame)
    print("sent!")
    
if __name__ == "__main__":
    rospy.init_node("custom_ik_solver", anonymous=True)
    s = rospy.Service("solve_ik", SolveIKSrv, callback)

    robot_urdf = URDF.from_parameter_server()

    # pub_orange = rospy.Publisher("orange_pub", PointStamped, queue_size=10)
    # pub_pink = rospy.Publisher("pink_pub", PoseStamped, queue_size=10)
    # pub_brown = rospy.Publisher("brown_pub", PointStamped, queue_size=10)
    # pub_purple = rospy.Publisher("purple_pub", PointStamped, queue_size=10)

    # pub_array = []
    
    # testIK()

    print("Listening for IK requests...")
    rospy.spin()
