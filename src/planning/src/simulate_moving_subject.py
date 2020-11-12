#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.msg import ModelState

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    SetModelState,
    GetModelState
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Twist
)

import baxter_interface

import numpy as np

def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       ball_pose=Pose(position=Point(x=0.4225, y=-0.1265, z=0.7725)),
                       ball_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('planning')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Ball URDF
    ball_xml = ''
    with open (model_path + "ball/model.urdf", "r") as ball_file:
        ball_xml=ball_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Ball URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("ball", ball_xml, "/",
                               ball_pose, ball_reference_frame)

        move_ball = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        get_ball = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        rate = rospy.Rate(3)
        while True:
            new_velocity = Twist()
            new_velocity.linear.x = np.random.uniform(-1,1)
            new_velocity.linear.y = np.random.uniform(-1,1)

            curr_state = get_ball("ball", ball_reference_frame)
            next_state = ModelState("ball", curr_state.pose, new_velocity, ball_reference_frame)
            move_resp = move_ball(next_state) 

            print(move_resp)
            rate.sleep()

    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("table")
        resp_delete = delete_model("ball")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

def main():
    """SDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Sawyer will eventually mis-pick or drop the ball. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # rospy.spin()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

if __name__ == '__main__':
    sys.exit(main())
