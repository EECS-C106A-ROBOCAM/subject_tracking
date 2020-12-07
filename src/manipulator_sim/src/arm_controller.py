#!/usr/bin/env python
import sys
import rospy
import tf2_ros
import traceback
import numpy as np

from baxter_interface import Limb

from tf2_geometry_msgs import do_transform_pose
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import (    
	Point,
	Quaternion,
	Pose,
	PoseStamped
)

from path_planner import PathPlanner

def move_to_goal_absolute(planner, x, y, z, orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
	while not rospy.is_shutdown():
		try:
			goal = PoseStamped()
			goal.header.frame_id = "base"

			#x, y, and z position
			goal.pose.position.x = x
			goal.pose.position.y = y
			goal.pose.position.z = z

			#Orientation as a quaternion
			goal.pose.orientation.x = or_x
			goal.pose.orientation.y = or_y
			goal.pose.orientation.z = or_z
			goal.pose.orientation.w = or_w

			plan = planner.plan_to_pose(goal, orien_const)

			# Might have to edit this for part 5
			if not planner.execute_plan(plan):
				raise Exception("Execution failed")
		except Exception as e:
			print e
			traceback.print_exc()
		else:
			break

def move_to_goal_relative(delta_pose, args):
	planner, transform_cam_to_hand, transform_hand_to_cam, orien_const = args[0], args[1], args[2], args[3]
	while not rospy.is_shutdown():
		try:
			curr_pose = do_transform_pose(planner.get_current_pose(), transform_hand_to_cam).pose
			
			goal = PoseStamped()
			goal.header.frame_id = "base"

			#x, y, and z position
			goal.pose.position.x = curr_pose.position.x + delta_pose.position.x
			goal.pose.position.y = curr_pose.position.y + delta_pose.position.y
			goal.pose.position.z = curr_pose.position.z + delta_pose.position.z

			#Orientation as a quaternion
			goal.pose.orientation.x += curr_pose.orientation.x
			goal.pose.orientation.y += curr_pose.orientation.y
			goal.pose.orientation.z += curr_pose.orientation.z
			goal.pose.orientation.w += curr_pose.orientation.w

			goal = do_transform_pose(goal, transform_cam_to_hand).pose

			plan = planner.plan_to_pose(goal, orien_const)

			# Might have to edit this for part 5
			if not planner.execute_plan(plan):
				raise Exception("Execution failed")
		except Exception as e:
			print e
			traceback.print_exc()
		else:
			break

def main():
	planner = PathPlanner("left_arm")

	pose = PoseStamped()
	pose.header.frame_id = "world"
	pose.pose.position.x = 0.5
	pose.pose.position.y = 0.0
	pose.pose.position.z = 0.0
	pose.pose.orientation.x = 0.0
	pose.pose.orientation.y = 0.0
	pose.pose.orientation.z = 0.0
	pose.pose.orientation.w = 1.0

	planner.add_box_obstacle(np.array([0.4, 1.2, 0.1]), "table", pose)

	tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	transform_cam_to_hand, transform_hand_to_cam = None, None
	while not transform_cam_to_hand and not transform_hand_to_cam:
		try:
			transform_cam_to_hand = tf_buffer.lookup_transform('left_hand', 'left_hand_camera', rospy.Time(0))
			transform_hand_to_cam = tf_buffer.lookup_transform('left_hand_camera', 'left_hand', rospy.Time(0))

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			continue

	# initialize
	move_to_goal_absolute(planner, x=0.75, y=0, z=0.48)
	print('Ready!')

	while not rospy.is_shutdown():
		rospy.Subscriber("camera_targets", Pose, move_to_goal_relative, (planner, transform_cam_to_hand, transform_hand_to_cam, []))


		

if __name__ == '__main__':
	rospy.init_node('arm_controller_node')
	main()
