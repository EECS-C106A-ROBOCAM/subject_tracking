#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>

#include <sensor_msgs/JointState.h>
#include "ik_solver/SolveIK.h"

bool solveIK(ik_solver::SolveIK::Request& req, ik_solver::SolveIK::Response& res) {
  TRAC_IK::TRAC_IK tracik_solver("base", "link_roll", "/robot_description", 0.5);

  ROS_INFO("Created solver");

  auto pose = req.input_pose.pose;
  KDL::Frame target_frame;
  tf::poseMsgToKDL(pose, target_frame);

  ROS_INFO("About to loop through inputs");
  KDL::JntArray input_joints{6};
  for (int i = 0; i < 6; ++i) {
    input_joints(i) = req.input_joints.position[i];
  }

  ROS_INFO("Going to solve now");
  KDL::JntArray output_joints;
  auto r = tracik_solver.CartToJnt(input_joints, target_frame, output_joints);
  if (r != 0) {
    ROS_ERROR_STREAM("Failed to find solution! " << r);
    return false;
  }

  ROS_INFO("Saving solution");
  res.output_joints.position = {0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 6; ++i) {
    res.output_joints.position[i] = output_joints(i);
  }
  ROS_INFO("All done!");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_solver");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("solve_ik", solveIK);
  ROS_INFO("IK Solver online"); 
  ros::spin();

  return 0;
}
