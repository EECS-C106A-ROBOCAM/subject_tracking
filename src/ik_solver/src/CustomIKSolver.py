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


def solveIK(targetPoseStamped):
    targetFrame = posemath.fromMsg(targetPoseStamped.pose) # Target pose as KDL frame

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

    camPitchToYaw = PyKDL.Frame(PyKDL.Vector(0, 0.03, 0.02)) # TODO(JS): Get these constants [m] from URDF/CAD
    camYawToRoll = PyKDL.Frame(PyKDL.Vector(0, 0.02, 0.02))
    camRollToCam = PyKDL.Frame(PyKDL.Vector(0, 0, 0.025))

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
    

    # print("Camera orientation: {}".format(cameraFrame.M.GetQuaternion()))

    # 3. Convert orange point to cylindrical coordinates
    orange_X, orange_Y, orange_Z = orangePoint
    orange_R = math.sqrt(orange_X ** 2 + orange_Y ** 2)
    orange_Theta = math.atan2(orange_Y, orange_X) # Theta measured from global positive X axis
    # 3. Complete: (above)
    print("Orange R: {} Theta: {}".format(orange_R, math.degrees(orange_Theta)))

    # 4. Solve for J2 and J3 in the idealized R-Z plane
    targetPointOrig_RZ = np.array([0.50, 0.12]) #np.array([orange_R, orange_Z])

    # First, remove the fixed offsets from the wrist, elbow, and shoulder pieces
    wristOffset_RZ = np.array([0.025, -0.045]) # TODO(JS): Get this from CAD/URDF
    elbowOffset_RZ = np.array([0.035, 0.01])
    shoulderOffset_RZ = np.array([-0.02, 0.12])

    targetPoint_RZ = targetPointOrig_RZ - wristOffset_RZ - elbowOffset_RZ - shoulderOffset_RZ
    print("Need to solve 2D 2-link IK from (0, 0) to ({}, {})".format(*targetPoint_RZ))

    # The following steps use the same labels as the classic 2D planar IK derivation
    a1, a2 = 0.3, 0.3
    ik_x, ik_y = targetPoint_RZ
    
    q2_a = math.acos((ik_x ** 2 + ik_y ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2))
    q1_a = math.atan2(ik_y, ik_x) - math.atan2(a2 * math.sin(-q2_a), a1 + a2 * math.cos(-q2_a))

    q2_b = -1 * math.acos((ik_x ** 2 + ik_y ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2))
    q1_b = math.atan2(ik_y, ik_x) + math.atan2(a2 * math.sin(q2_b), a1 + a2 * math.cos(q2_b))

    # Choose 'better' set of q1_ab, q2_ab
    q1, q2 = q1_a, q2_a # TODO(JS): Is this always the better one?

    J2_initial = q1
    J2_offset = math.radians(90) # J3's zero position is vertical, not horizontal
    J2 = J2_initial - J2_offset

    # Since we have a parallel link, the angle for J3 is not simply q2. Instead, use transversal
    J3_initial = q1 - q2
    J3_offset = math.radians(-60) # J2's zero position is below horizontal
    J3 = J3_initial - J3_offset
    # 4. Complete (above)
    print("To reach simple (R, Z) = ({}, {}), set J2={} J3={}".format(targetPoint_RZ[0], targetPoint_RZ[1], math.degrees(J2), math.degrees(J3)))
    
    
    

def testIK():
    pub = rospy.Publisher("target_pose", PoseStamped, queue_size=10)

    """ CHANGE ME """
    pitch_deg, yaw_deg, roll_deg = 0, 0, 0
    x, y, z = 0.2, 0.4, 0.2 # in meters

    targetVector = PyKDL.Vector(x, y, z)

    pitch, yaw, roll = math.radians(pitch_deg), math.radians(yaw_deg), math.radians(roll_deg)
    targetQuat = tf.transformations.quaternion_from_euler(pitch, yaw, roll, axes='rxzy')

    print("Input quaternion: {}".format(targetQuat))
    targetRotation = PyKDL.Rotation.Quaternion(*targetQuat)

    targetFrame = PyKDL.Frame(targetRotation, targetVector)
    targetPose = posemath.toMsg(targetFrame)

    targetPoseStamped = PoseStamped()
    targetPoseStamped.pose = targetPose

    pub.publish(targetPoseStamped)
    print("sent!")


if __name__ == "__main__":
    rospy.init_node("custom_ik_solver", anonymous=True)
    rospy.Subscriber("target_pose", PoseStamped, solveIK)

    print("Listening for IK requests...")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        testIK()
        rate.sleep()