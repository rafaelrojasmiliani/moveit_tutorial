
// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "grasp_tools.h"

#include <moveit/move_group_interface/move_group_interface.h>
// moveit::planning_interface::MoveGroupInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// moveit::planning_interface::PlanningSceneInterface
//
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "grasp_filter_demo");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node_handle;

  // ---------------------------------------
  // 2. Instantate moveit stuff
  // ---------------------------------------
  // 2.1 Robot Model loader
  robot_model_loader::RobotModelLoader my_robot_model_loader(
      "robot_description");
  // 2.2 Robot Model
  const robot_model::RobotModelPtr &robot_model =
      my_robot_model_loader.getModel();
  // 2.3 MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface mgi("arm");
  // 2.4 PlanningSceneInterface
  moveit::planning_interface::PlanningSceneInterface psi;

  // -------------------
  // 3. Instantiate visual tools
  // ---------------------------
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools =
      std::make_shared<moveit_visual_tools::MoveItVisualTools>(
          robot_model->getModelFrame(), "/rviz_visual_tools");

  // -------------------
  // 4. Instantiate Moveit Grasp stuff
  // ---------------------------
  moveit_grasps::SuctionGraspScoreWeightsPtr grasp_scores;
  moveit_grasps::SuctionGraspDataPtr grasp_data;
  moveit_grasps::SuctionGraspGeneratorPtr grasp_generator;

  grasp_data = get_grasp_data_initialized_from_parameters(
      node_handle, "end_effector", robot_model);

  grasp_scores = get_grasp_suction_score(1, 1, 1, 2, 2, 2, 10);
  std::vector<double> ideal_orientation = {3.14, 0.0, 0.0};

  grasp_generator = get_initialized_grasp_generator(visual_tools, grasp_scores,
                                                    ideal_orientation);

  return 0;
}
