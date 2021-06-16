#ifndef SIMPLEMANIPULATOR_H
#define SIMPLEMANIPULATOR_H
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h> // PlanningSceneMonitor
#include <moveit/robot_state/robot_state.h> // robot_state::RobotStatePtr, JointModelGroup
#include <moveit/trajectory_processing/iterative_time_parameterization.h> // IterativeParabolicTimeParameterization
// moveit::planning_interface::MoveGroupInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// moveit::planning_interface::PlanningSceneInterface
#include "objectdescription.hpp"
#include <moveit_grasps/grasp_planner.h>
#include <moveit_grasps/suction_grasp_filter.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
class SimpleManipulator {
private:
  robot_model_loader::RobotModelLoader robot_model_loader_;
  const robot_model::RobotModelPtr &robot_model_;
  const robot_state::RobotStatePtr robot_state_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  moveit::planning_interface::MoveGroupInterface mgi_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  const moveit::core::JointModelGroup *const arm_jmg_;
  const moveit::core::JointModelGroup *const ee_jmg_;
  const std::string arm_flange_link_name_;

  //   parent_link_ =
  //   robot_model_->getLinkModel(ee_jmg_->getEndEffectorParentGroup().second);
  //

  moveit_grasps::SuctionGraspFilterPtr grasp_filter_;
  moveit_grasps::GraspPlannerPtr grasp_planner_;

  trajectory_processing::IterativeParabolicTimeParameterization
      parabolic_parametrizer_;
  SimpleManipulator(const SimpleManipulator &that);
  SimpleManipulator &operator=(const SimpleManipulator &);

  bool parametrize_path(
      const std::vector<robot_state::RobotStatePtr> _robot_state_array,
      moveit_msgs::RobotTrajectory &_trj_msg);

public:
  SimpleManipulator(const std::string &_group_name,
                    const std::string &_ee_group_name);

  bool pick(ObjectDescription &object);
  virtual ~SimpleManipulator();
};

#endif /* SIMPLEMANIPULATOR_H */
