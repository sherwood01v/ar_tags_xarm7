
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  // Basic ROS setup
  // ^^^^^^^^^^^^^^^
  ros::init(argc, argv, "pose_goal");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveGroupInterface API can be found at:
  // http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html

  // MoveIt setup
  // ^^^^^^^^^^^^
  // A MoveGroupInterface instance (a client for the MoveGroup action) can be easily setup using just the name of the
  // planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group("xarm7");

  // Planning to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group by defining a target pose for the end-effector.
  // As pose requires 7 variables - x, y, and z for the position and x, y, z, and w for the orientation- to be defined,
  // it is convenient to first get the current end-effector pose and then modify it to a suitable target pose.

  // Getting the current pose of the end-effector.
  geometry_msgs::PoseStamped current_pose;
  current_pose = move_group.getCurrentPose();

  // Modifying the current pose into a target pose.
  // Feel free to change the following position values to see their influence on the motion planning
  geometry_msgs::Pose target_pose = current_pose.pose;
  target_pose.position.x += 0.1;
  target_pose.position.y += -0.2;
  target_pose.position.z += -0.0;
  // Setting the target pose for the end-effector
  move_group.setPoseTarget(target_pose);

  // Calling the planner to compute the motion plan, which is then stored in my_plan.
  // Note that we are just planning, not asking MoveGroupInterface to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
  if (success)
  {
    ROS_INFO("[movegroup_interface_demo/pose_goal] Planning OK. Proceeding.");
  }
  else
  {
    ROS_WARN("[movegroup_interface_demo/pose_goal] Planning failed. Shutting Down.");
    ros::shutdown();
    return 0;
  }
  
  // Executing the computed plan
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Uncomment the following line to make the robot move according to the computed plan
  // move_group.execute(my_plan);

  ros::shutdown();
  return 0;
}