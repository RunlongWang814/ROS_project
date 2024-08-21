#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_interface/planning_interface.h>

// define function detectingPosition
bool detectingPosition(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // get current joint state
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = 
      current_state->getJointModelGroup("arm");

  // set traget radius of the joint
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = 0.0;   // Joint 1
  joint_group_positions[1] = 0.5;   // Joint 2
  joint_group_positions[2] = -0.5;  // Joint 3
  joint_group_positions[3] = 1.5;   // Joint 4

  // set the joint postion
  move_group.setJointValueTarget(joint_group_positions);

  // plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // execute
  if (success)
  {
    ROS_INFO("Moving to detecting position.");
    move_group.move();
  }
  else
  {
    ROS_ERROR("Planning to detecting position failed.");
  }

  return success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_to_fixed_position_example");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // init moveit move group
  moveit::planning_interface::MoveGroupInterface move_group("arm");

  // planning time 10s
  move_group.setPlanningTime(10.0);

  // call function to move to detectiong positions
  bool success = detectingPosition(move_group);

  if(success)
  {
    ROS_INFO("Successfully moved to the detecting position.");
  }
  else
  {
    ROS_ERROR("Failed to move to the detecting position.");
  }

  ros::shutdown();
  return 0;
}
