#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int pickup_x=0;
int pickup_y=0;

int drop_x=5;
int drop_y=5;

int main(int argc, char **argv) {
  bool reach_pickup = false;
  bool reach_drop_off = false;
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pickup_x;
  goal.target_pose.pose.position.y = pickup_y;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, reach pickup");
    reach_pickup = true;
  }else{
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }

  // reach pickup zone, wait for next command
  ros::Duration(5).sleep();

  goal.target_pose.pose.position.x = drop_x;
  goal.target_pose.pose.position.y = drop_y;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, reach drop off");
    reach_drop_off = true;
  }else{
    ROS_INFO("The base failed to move to the drop off");
  }

  if(reach_pickup&&reach_drop_off){
    ROS_INFO("Success!");
  }
  return 0;
}