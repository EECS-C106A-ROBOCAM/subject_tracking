#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/JointState.h>
#include "ik_solver/SolveIKSrv.h"

bool solveIK(ik_solver::SolveIKSrv::Request& req, ik_solver::SolveIKSrv::Response& res) {

  tf::Transform targetPose;
  tf::poseMsgToTF(req.input_pose.pose, targetPose);

  // 1. Use target orientation to solve J4, J5, J6
  // First, convert quaternion into Euler angles in X, Z, Y order
  auto targetQuat = targetPose.getRotation();

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
