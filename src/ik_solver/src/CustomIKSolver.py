#!/usr/bin/env python

import tf
import rospy
from tf_conversions import posemath
import PyKDL
import math
import numpy as np

from geometry_msgs.msg import (
    PoseStamped
)

from sensor_msgs.msg import (
    JointState
)

from ik_solver.srv import (
    SolveIKSrv
)

def callback(req):
    targetFrame = posemath.fromMsg(req.input_pose.pose) # Target pose as KDL frame


def solveIK(targetFrame):
    """ CONSTANTS """
    BASE_TO_BASE_YAW = PyKDL.Vector(0, -0.032, 0.083)           # Correct
    BASE_YAW_TO_BOTTOM_4B = PyKDL.Vector(0, 0.012, 0.041)       # Correct
    BOTTOM_4B_TO_BOTTOM_4B_END = PyKDL.Vector(0, 0, 0.336)      # Correct
    BOTTOM_4B_END_TO_TOP_4B = PyKDL.Vector(0, 0.030, 0.026)     # Correct
    TOP_4B_TO_TOP_4B_END = PyKDL.Vector(0, 0.336, 0)            # Correct
    TOP_4B_END_TO_CAMERA_PITCH = PyKDL.Vector(0, 0.046, 0.006)  # Correct
    CAMERA_PITCH_TO_CAMERA_YAW = PyKDL.Vector(0, 0.039, 0.013)  # Correct
    CAMERA_YAW_TO_CAMERA_ROLL = PyKDL.Vector(0, 0.021, 0.017)   # Correct  
    CAMERA_ROLL_TO_CAMERA = PyKDL.Vector(0, 0.008, 0.024)       # Correct
    JOINT_1_OFFSET_DEG = 90                                     # Correct
    JOINT_2_OFFSET_DEG = 90                                     # Correct
    JOINT_3_OFFSET_DEG = -30                                    # Correct


    # 1. Solve for J4, J5_initial, J6
    # First, convert quaternion orientation to XZY order Euler angles
    targetQuat = targetFrame.M.GetQuaternion() # Get quaternion from KDL frame (x, y, z, w)
    pitch, yaw, roll = tf.transformations.euler_from_quaternion(targetQuat, axes='rxzy')

    pitch_deg, yaw_deg, roll_deg = math.degrees(pitch), math.degrees(yaw), math.degrees(roll)

    # 1. Complete:
    J4, J5_initial, J6 = pitch, yaw, roll

    # 2. Determine position of orange point
    # First, construct KDL chain of the 3 links involved in J4-J6
    cameraOffsetChain = PyKDL.Chain()

    cameraPitchJoint = PyKDL.Joint(PyKDL.Joint.RotX)
    cameraYawJoint = PyKDL.Joint(PyKDL.Joint.RotZ)
    cameraRollJoint = PyKDL.Joint(PyKDL.Joint.RotY)

    camPitchToYaw = PyKDL.Frame(CAMERA_PITCH_TO_CAMERA_YAW)
    camYawToRoll = PyKDL.Frame(CAMERA_YAW_TO_CAMERA_ROLL)
    camRollToCam = PyKDL.Frame(CAMERA_ROLL_TO_CAMERA)

    cameraPitchSegment = PyKDL.Segment(cameraPitchJoint, camPitchToYaw)
    cameraYawSegment = PyKDL.Segment(cameraYawJoint, camYawToRoll)
    cameraRollSegment = PyKDL.Segment(cameraRollJoint, camRollToCam)

    cameraOffsetChain.addSegment(cameraPitchSegment)
    cameraOffsetChain.addSegment(cameraYawSegment)
    cameraOffsetChain.addSegment(cameraRollSegment)

    # Next, find XYZ offset from end of top four bar to camera
    cameraJointAngles = PyKDL.JntArray(3)
    cameraJointAngles[0], cameraJointAngles[1], cameraJointAngles[2] = J4, J5_initial, J6
    cameraOffsetChainFK = PyKDL.ChainFkSolverPos_recursive(cameraOffsetChain)
    cameraFrame = PyKDL.Frame()
    cameraOffsetChainFK.JntToCart(cameraJointAngles, cameraFrame)

    # Now, find XYZ offset from end of top four bar to the camera yaw joint
    cameraPitchChain = PyKDL.Chain()
    cameraPitchChain.addSegment(cameraPitchSegment)

    cameraPitchJointAngle = PyKDL.JntArray(1)
    cameraPitchJointAngle[0] = J4

    cameraPitchChainFK = PyKDL.ChainFkSolverPos_recursive(cameraPitchChain)
    cameraYawFrame = PyKDL.Frame()
    cameraPitchChainFK.JntToCart(cameraPitchJointAngle, cameraYawFrame)

    # Use these two offsets to find the newly-fixed XYZ displaement from camera yaw to camera
    cameraYawToCameraDisplacement = cameraFrame.p - cameraYawFrame.p

    # 2. Complete:
    orangePoint = targetFrame.p - cameraYawToCameraDisplacement
    
    # print("Camera offset: {}".format(cameraFrame.p))
    # print("Camera Yaw offset: {}".format(cameraYawFrame.p))
    # print("CameraYaw-to-Camera Fixed XYZ displacement: {}".format(cameraYawToCameraDisplacement))
    print("Orange Point: {}".format(orangePoint))
    print("Camera orientation: {}".format(cameraFrame.M.GetQuaternion()))

    # 3. Convert orange point to cylindrical coordinates
    orange_X, orange_Y, orange_Z = orangePoint
    orange_R = math.sqrt(orange_X ** 2 + orange_Y ** 2)
    orange_Theta = math.atan2(orange_Y, orange_X) # Theta measured from global positive X axis
    # 3. Complete: (above)

    # print("Orange R: {} Theta: {}".format(orange_R, math.degrees(orange_Theta)))

    # 4. Solve for J2 and J3 in the idealized R-Z plane
    targetPointOrig_RZ = np.array([orange_R, orange_Z])

    # First, remove the fixed offsets from the wrist, elbow, and shoulder pieces
    staticWristOffset_RZ = np.array([TOP_4B_END_TO_CAMERA_PITCH.y(), TOP_4B_END_TO_CAMERA_PITCH.z()])
    elbowOffset_RZ = np.array([BOTTOM_4B_END_TO_TOP_4B.y(), BOTTOM_4B_END_TO_TOP_4B.z()])
    shoulderOffset_RZ = np.array([BASE_TO_BASE_YAW.y() + BASE_YAW_TO_BOTTOM_4B.y(), BASE_TO_BASE_YAW.z() + BASE_YAW_TO_BOTTOM_4B.z()])

    # Also remove the dynamic offset from the camera pitch link 
    cameraPitchOffset_global = PyKDL.Frame(PyKDL.Rotation.RotX(J4)) * CAMERA_PITCH_TO_CAMERA_YAW 
    cameraPitchOffset_RZ =  np.array([cameraPitchOffset_global.y(), cameraPitchOffset_global.z()])
    print("Camera Pitch Offset: {}".format(cameraPitchOffset_RZ))


    targetPoint_RZ = targetPointOrig_RZ - staticWristOffset_RZ - elbowOffset_RZ - shoulderOffset_RZ - cameraPitchOffset_RZ

    # print("Need to solve 2D 2-link IK from (0, 0) to ({}, {})".format(*targetPoint_RZ))

    # The following steps use the same labels as the classic 2D planar IK derivation
    a1, a2 = BOTTOM_4B_TO_BOTTOM_4B_END.z(), TOP_4B_TO_TOP_4B_END.y()
    ik_x, ik_y = targetPoint_RZ
    
    q2_a = math.acos((ik_x ** 2 + ik_y ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2))
    q1_a = math.atan2(ik_y, ik_x) - math.atan2(a2 * math.sin(-q2_a), a1 + a2 * math.cos(-q2_a))

    q2_b = -1 * math.acos((ik_x ** 2 + ik_y ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2))
    q1_b = math.atan2(ik_y, ik_x) + math.atan2(a2 * math.sin(q2_b), a1 + a2 * math.cos(q2_b))

    # Choose 'better' set of q1_ab, q2_ab
    q1, q2 = q1_a, q2_a # TODO(JS): Is this always the better one?

    J2_initial = q1
    J2_offset = math.radians(JOINT_2_OFFSET_DEG) # J3's zero position is vertical, not horizontal
    J2 = J2_initial - J2_offset

    # Since we have a parallel link, the angle for J3 is not simply q2. Instead, use transversal
    J3_initial = q1 - q2
    J3_offset = math.radians(JOINT_3_OFFSET_DEG) # J2's zero position is below horizontal
    J3 = J3_initial - J3_offset
    # 4. Complete (above)

    # print("To reach simple (R, Z) = ({}, {}), set J2={} J3={}".format(targetPoint_RZ[0], targetPoint_RZ[1], math.degrees(J2), math.degrees(J3)))
    
    # 5. Use the Theta from cylindrical coordinates as the J1 angle, and update J5 accordingly
    J1 = orange_Theta - math.radians(JOINT_1_OFFSET_DEG)
    J5 = J5_initial - orange_Theta
    # 5. Complete (above)

    jointAngles = [J1, J2, J3, J4, J5, J6]
    jointAngles_deg = [math.degrees(j) for j in jointAngles]
    print("Final joint angles: {}".format(jointAngles_deg))

    # 6. (optional) Sanity check on solution:
    completeArmChain = PyKDL.Chain()
    
    baseYawJoint = PyKDL.Joint(PyKDL.Joint.RotZ)
    bot4BJoint = PyKDL.Joint(PyKDL.Joint.RotX)
    top4BJoint = PyKDL.Joint(PyKDL.Joint.RotX)

    # Use Fake joints to artificially create 4 bar parallel constraint
    bot4BFakeJoint = PyKDL.Joint(PyKDL.Joint.RotX)
    top4BFakeJoint = PyKDL.Joint(PyKDL.Joint.RotX)

    baseToBaseYaw = PyKDL.Frame(BASE_TO_BASE_YAW)
    baseYawToBot4B = PyKDL.Frame(BASE_YAW_TO_BOTTOM_4B)
    """
    bot4BToBot4BFake = PyKDL.Frame(PyKDL.Vector(0, 0, a1))
    bot4BFakeToTop4B = PyKDL.Frame(PyKDL.Vector(0, elbowOffset_RZ[0], elbowOffset_RZ[1]))
    top4BToTop4BFake = PyKDL.Frame(PyKDL.Vector(0, a2 * math.cos(J3_offset), a2 * math.sin(J3_offset)))
    """
    # TODO(JS): Remove need for this magic stuff
    fixedBaseJoint = PyKDL.Joint(PyKDL.Joint.RotX) # won't actually spin
    magic4BJoint = PyKDL.Joint(PyKDL.Joint.RotX) # won't actually sping
    magic4BFrame = PyKDL.Frame(targetFrame.p - cameraFrame.p)
    
    baseSegment = PyKDL.Segment(fixedBaseJoint, baseToBaseYaw)
    baseYawSegment = PyKDL.Segment(baseYawJoint, baseYawToBot4B)
    magic4BSegment = PyKDL.Segment(magic4BJoint, magic4BFrame)

    completeArmChain.addSegment(baseSegment)
    completeArmChain.addSegment(baseYawSegment)
    completeArmChain.addSegment(magic4BSegment)
    completeArmChain.addChain(cameraOffsetChain)

    completeArmChainFK = PyKDL.ChainFkSolverPos_recursive(completeArmChain)

    solvedJoints = PyKDL.JntArray(6)
    solvedJoints[0] = 0 # Fake joint
    solvedJoints[1] = J1
    solvedJoints[2] = 0 # Fake joint
    solvedJoints[3] = J4
    solvedJoints[4] = J5
    solvedJoints[5] = J6

    print("# Joints: {} # Segments: {}".format(completeArmChain.getNrOfJoints(), completeArmChain.getNrOfSegments()))
    
    producedFrame = PyKDL.Frame()
    rc = completeArmChainFK.JntToCart(solvedJoints, producedFrame)
    print("Result: {}".format(rc))
    print("Output position: {}\nExpected position: {}".format(producedFrame.p, targetFrame.p))
    print("Output orientation: {}\nExpected orientation: {}".format(producedFrame.M, targetFrame.M))

    # 7. Create JointState message for return
    ret = JointState()
    ret.header.stamp = rospy.Time.now()
    ret.position = jointAngles

    return ret
    

def testIK():
    """ CHANGE ME """
    pitch_deg, yaw_deg, roll_deg = 0, 0, 0 #np.random.uniform(-90, 90), np.random.uniform(-90, 90), np.random.uniform(-90, 90)
    x, y, z = 0.0, 0.4, 0.2 # in meters

    targetVector = PyKDL.Vector(x, y, z)

    pitch, yaw, roll = math.radians(pitch_deg), math.radians(yaw_deg), math.radians(roll_deg)
    targetQuat = tf.transformations.quaternion_from_euler(pitch, yaw, roll, axes='rxzy')

    print("Input quaternion: {}".format(targetQuat))
    targetRotation = PyKDL.Rotation.Quaternion(*targetQuat)

    targetFrame = PyKDL.Frame(targetRotation, targetVector)

    jointAngles = solveIK(targetFrame)
    print("sent!")


if __name__ == "__main__":
    rospy.init_node("custom_ik_solver", anonymous=True)
    s = rospy.Service("solve_ik", SolveIKSrv, callback)

    testIK()

    print("Listening for IK requests...")
    rospy.spin()
