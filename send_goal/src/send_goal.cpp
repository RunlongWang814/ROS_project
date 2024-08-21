#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define the long code to MoveBaseClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");

    // init the MoveBase action client
    MoveBaseClient ac("move_base", true);

    // Wait for the move_base 
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Define the goal
    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters for the goal
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Set the goal position
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal to the move_base action server
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait for the result of the navigation
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The robot moved to the goal successfully");
    else
        ROS_INFO("The robot failed to move to the goal");

    return 0;
}
