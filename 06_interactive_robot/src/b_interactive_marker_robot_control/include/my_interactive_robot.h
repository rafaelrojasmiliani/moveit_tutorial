#ifndef MY_INTERACTIVE_ROBOT_H
#define MY_INTERACTIVE_ROBOT_H

#include <interactive_markers/interactive_marker_server.h>
#include <moveit/robot_model/robot_model.h> //robot_model::RobotModel
#include <moveit/robot_model_loader/robot_model_loader.h> // robot_model_loader::RobotModelLoader
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h> //planning_scene::PlanningScene
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h> // NodeHandle
class MyInteractiveRobot {
private:
  moveit_visual_tools::MoveItVisualTools visualizer_;
  robot_model_loader::RobotModelLoader loader_;
  const robot_model::RobotModelPtr model_;
  robot_model::RobotStatePtr robot_state_display_;
  planning_scene::PlanningScene workspace_;
  const robot_model::JointModelGroup *const joint_group_;
  // base ross stuff
  ros::NodeHandle nh_;
  // Names
  const std::string link_name_;
  const std::string group_name_;
  const std::string base_link_name_;
  MyInteractiveRobot(const MyInteractiveRobot &that);
  MyInteractiveRobot &operator=(const MyInteractiveRobot &);

public:
  MyInteractiveRobot(const std::string &_base_link_name,
                     const std::string &_group_name,
                     const std::string &_link_name);
  virtual ~MyInteractiveRobot() {}
  void spin();
  bool initialize();
  void set_robot_state_display(const Eigen::Isometry3d &_pose);
};

#endif /* MY_INTERACTIVE_ROBOT_H */
