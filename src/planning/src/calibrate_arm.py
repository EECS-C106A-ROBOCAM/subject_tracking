#!/usr/bin/env python

import baxter_interface
import rospy
import ik_solver

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import (    
    Point,
    Quaternion,
    Pose
)

# Moves relative to current Position
def moveRel(x,y,z, limb):
    pose = left.endpoint_pose()
    
    curX = pose["position"].x
    curY = pose["position"].y
    curZ = pose["position"].z

    newX = curX + x
    newY = curY + y
    newZ = curZ + z

    goal = Point(newX,newY,newZ)
    orient = Quaternion( 0.0, 0.0, 0.5, 0.0)
    limb_joints = ik_solver.ik_solve('left', goal, orient)
    
    if limb_joints != -1:
        print "limb_joints: ", limb_joints
        print "moving arm to limb_joints joints"
        left.move_to_joint_positions(limb_joints)


# Takes a Point()
def moveTo(goal, limb):
    orient = Quaternion(x=0.8742482583887288, y=0.80385547999510975, z=0.2525354310395808353, w=0)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        tranisform = tf_buffer.lookup_transform('left_hand',
                                                'left_hand_camera',     # source frame
                                                rospy.Time(0))   # get the tf at first available time
        goal = tf2_geometry_msgs.do_transform_pose(Pose(goal, orient), transform).pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print 'UH OH UH OH'
        continue

    limb_joints = ik_solver.ik_solve('left', goal, orient)

    if limb_joints != -1:
        left.move_to_joint_positions(limb_joints)
        print "done!"



def main():
    rospy.init_node('move_left_hand')
    rs = baxter_interface.RobotEnable()
    rs.enable()

    left = baxter_interface.Limb('left')
    left.set_joint_position_speed(1.0)

    print "-----------------------------------"
    pose = left.endpoint_pose()
    print "current pose: ", pose
    moveTo(Point(x=0.5568081191424449, y=0.14277077923725493, z=0.48684692613282426), left)
    print left.endpoint_pose()

if __name__ == '__main__':
    main()