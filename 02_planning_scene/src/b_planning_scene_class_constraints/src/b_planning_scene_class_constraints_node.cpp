/* Author: Rafael A. Rojas*/

#include <ros/ros.h> // ros::init, ros::AsyncSpinner, 
#include <sensor_msgs/JointState.h> // sensor_msgs::JointState
// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "a_planning_scene_class_example");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::size_t count = 0;
  ros::NodeHandle nh;

  ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("joint_states_cmd", 1);
  sensor_msgs::JointState js;
  // 1) instantiate the  MoveIt robot model loader
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // 2) instantiate the  MoveIt PlanningScene with the moveit model
  robot_model::RobotModelPtr my_robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(my_robot_model);

    robot_state::RobotState& kinematic_model_actual = planning_scene.getCurrentStateNonConst();
    const robot_model::JointModelGroup* joint_group = kinematic_model_actual.getJointModelGroup("myrobotplanninggroup");
    std::string link_name = joint_group->getLinkModelNames().back();

  geometry_msgs::PoseStamped desired_pose;
  desired_pose.header.frame_id = "world";
  desired_pose.pose.orientation.w = 1.0;
  desired_pose.pose.position.x = 0.3;
  desired_pose.pose.position.y = -0.185;
  desired_pose.pose.position.z = 0.5;
  moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(link_name, desired_pose);

  bool constrained = planning_scene.isStateConstrained(kinematic_model_actual, goal_constraint);
  ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));

  ros::shutdown();
  return 0;
}

