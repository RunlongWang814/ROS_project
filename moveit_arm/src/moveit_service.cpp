#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_interface/planning_interface.h>
#include <link_service/BluePoint.h>  // include service

// Function to move the robot to a fixed detecting position
bool detectingPosition(moveit::planning_interface::MoveGroupInterface& move_group)
{
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = 
      current_state->getJointModelGroup("arm");

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = 0.0;   // Joint 1
  joint_group_positions[1] = 0.5;   // Joint 2
  joint_group_positions[2] = -0.5;  // Joint 3
  joint_group_positions[3] = 1.5;   // Joint 4

  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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

// Function to request the coordinates from the service and print them
bool getBluePointCoordinates(ros::NodeHandle& nh, float& x, float& y, float& z)
{
  ros::ServiceClient client = nh.serviceClient<link_service::BluePoint>("get_blue_point");
  
  ros::service::waitForService("get_blue_point");

  link_service::BluePoint srv;  

  if (client.call(srv))
  {
    x = srv.response.Y + 0.134; // calibration between the camera link and base link
    y = srv.response.X;
    z = 0.146;  
    ROS_INFO("Received coordinates: X: %f, Y: %f, Z: %f", x, y, z);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service get_blue_point");
    return false;
  }
}

// Function to move the robot to the specified coordinates
bool moveToBluePoint(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y, float z)
{
  geometry_msgs::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  move_group.setPositionTarget(
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);
  //plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.getCurrentState();
  
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    ROS_INFO("Moving to blue point position.");
    move_group.move(); //execult
  }
  else
  {
    ROS_ERROR("Planning to blue point position failed.");
  }

  return success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_with_service");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("arm"); 
  move_group.setPlanningTime(10.0);

  bool success = detectingPosition(move_group);

  if(success)
  {
    ROS_INFO("Successfully moved to the detecting position.");

    float x, y, z;
    while (ros::ok())
    {
      if (getBluePointCoordinates(node_handle, x, y, z))
      {
        if (moveToBluePoint(move_group, x, y, z))
        {
          ROS_INFO("Successfully moved to the blue point.");
          ros::Duration(2.0).sleep();  
        }
        else
        {
          ROS_WARN("Failed to move to the blue point, retrying...");
        }
      }
      else
      {
        ROS_ERROR("Failed to get blue point coordinates, retrying...");
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to move to the detecting position.");
  }

  ros::shutdown();
  return 0;
}
