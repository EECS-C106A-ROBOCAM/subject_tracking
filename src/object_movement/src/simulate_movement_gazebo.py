#!/usr/bin/env python

import rospy
import rospkg
import tf2_ros

from gazebo_msgs.msg import ModelState

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    SetModelState,
    GetModelState
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Twist,
    TransformStamped
)

import numpy as np

def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       ball_pose=Pose(position=Point(x=0.4225, y=-0.1265, z=0.7725)),
                       ball_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('object_movement')+"/models/"

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

def register_frame(pose):
    tf2Stamp = TransformStamped()
    tf2Stamp.header.stamp = rospy.Time.now()
    tf2Stamp.header.frame_id = 'world'
    tf2Stamp.child_frame_id = 'tracked_object'
    tf2Stamp.transform.translation.x, tf2Stamp.transform.translation.y, tf2Stamp.transform.translation.z = pose.position.x, pose.position.y, pose.position.z
    tf2Stamp.transform.rotation.x, tf2Stamp.transform.rotation.y, tf2Stamp.transform.rotation.z, tf2Stamp.transform.rotation.w = 0, 0, 0, 1#pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    tf2Broadcast.sendTransform(tf2Stamp)

def simulate_movement(reference_frame="world", max_speed=1):
    move_ball = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    get_ball = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rate = rospy.Rate(3)
    try:
        while not rospy.is_shutdown():

            speed = np.random.uniform(0, max_speed)
            theta = np.random.uniform(0, 360)

            twist = Twist()
            twist.linear.x = speed * np.cos(theta)
            twist.linear.y = speed * np.sin(theta)

            curr_state = get_ball("ball", reference_frame)
            next_state = ModelState("ball", curr_state.pose, twist, reference_frame)
            move_resp = move_ball(next_state) 
            register_frame(curr_state.pose)
            rate.sleep()

    except rospy.ROSInterruptException, e:
        print("ROS interrupt: {0}".format(e))

def main():
    rospy.init_node("movement_simulator")
    # Load gazebo models for simulation
    load_gazebo_models()

    # Register callback to delete models on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    print("Running simulation")
    simulate_movement(max_speed=3)

if __name__ == '__main__':
    tf2Broadcast = tf2_ros.TransformBroadcaster()
    
    main()
