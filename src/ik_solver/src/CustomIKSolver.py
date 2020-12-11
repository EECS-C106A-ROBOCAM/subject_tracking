#!/usr/bin/env python

import tf
import rospy
from tf_conversions import posemath
import PyKDL
import math

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
    
    print("Camera offset: {}".format(cameraFrame.p))
    print("Camera Yaw offset: {}".format(cameraYawFrame.p))
    print("CameraYaw-to-Camera Fixed XYZ displacement: {}".format(cameraYawToCameraDisplacement))
    print("Orange Point: {}".format(orangePoint))
    

    print("Camera orientation: {}".format(cameraFrame.M.GetQuaternion()))
    
    

def testIK():
    pub = rospy.Publisher("target_pose", PoseStamped, queue_size=10)

    """ CHANGE ME """
    pitch_deg, yaw_deg, roll_deg = 0, 0, 0
    x, y, z = 0.0, 0.4, 0.2 # in meters

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