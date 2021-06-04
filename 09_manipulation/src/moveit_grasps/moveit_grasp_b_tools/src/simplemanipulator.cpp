#include "simplemanipulator.hpp"
#include <string>

SimpleManipulator::SimpleManipulator(const std::string &_arm_group_name,
                                     const std::string &_ee_group_name)
    : robot_model_loader_("robot_description"),
      robot_model_(robot_model_loader_.getModel()),
      robot_state_(new robot_state::RobotState(robot_model_)),
      planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor(
          "robot_description")),
      visual_tools_(new moveit_visual_tools::MoveItVisualTools(
          robot_model_->getModelFrame(), "/rviz_visual_tools")),
      mgi_(_arm_group_name), psi_(),
      arm_jmg_(robot_model_->getJointModelGroup(_arm_group_name)),
      ee_jmg_(robot_model_->getJointModelGroup(_ee_group_name)),
      arm_flange_link_name_(ee_jmg_->getEndEffectorParentGroup().second),
      grasp_filter_(
          new moveit_grasps::SuctionGraspFilter(robot_state_, visual_tools_)),
      grasp_planner_(new moveit_grasps::GraspPlanner(visual_tools_)),
      parabolic_parametrizer_() {

  visual_tools_->loadMarkerPub();
  visual_tools_->loadRobotStatePub("/display_robot_state");
  visual_tools_->loadTrajectoryPub("/display_planned_path");
  visual_tools_->loadSharedRobotState();
  visual_tools_->enableBatchPublishing();
}

SimpleManipulator::~SimpleManipulator() {}

bool SimpleManipulator::pick(ObjectDescription &_object) {

  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
  std::vector<moveit_msgs::RobotTrajectory> pick_sequence;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  moveit::planning_interface::MoveItErrorCode moveit_error;

  planning_scene_monitor_->requestPlanningSceneState();

  _object.compute_grasp_candidates(grasp_candidates);

  grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_,
                              arm_jmg_, robot_state_, true, _object.get_name());

  moveit_grasps::GraspCandidatePtr &best_candidate = grasp_candidates[0];

  grasp_planner_->planApproachLiftRetreat(best_candidate, robot_state_,
                                          planning_scene_monitor_, false,
                                          _object.get_name());

  mgi_.setStartStateToCurrentState();
  mgi_.setJointValueTarget(
      *best_candidate->segmented_cartesian_traj_[moveit_grasps::APPROACH]
           .front());

  moveit_error = mgi_.plan(motion_plan);

  if (moveit_error != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    return false;
  }

  mgi_.execute(motion_plan);

  for (const moveit_grasps::GraspTrajectorySegments &stage :
       {moveit_grasps::APPROACH, moveit_grasps::LIFT, moveit_grasps::RETREAT}) {
    moveit_msgs::RobotTrajectory trajectory;
    parametrize_path(best_candidate->segmented_cartesian_traj_[stage],
                     trajectory);
    mgi_.setStartStateToCurrentState();
    mgi_.execute(trajectory);
    if (stage == moveit_grasps::APPROACH)
      mgi_.attachObject(_object.get_name());
  }

  return true;
}

bool SimpleManipulator::parametrize_path(
    const std::vector<robot_state::RobotStatePtr> _robot_state_array,
    moveit_msgs::RobotTrajectory &_trj_msg) {

  robot_trajectory::RobotTrajectory trajectory(robot_model_,
                                               arm_jmg_->getName());

  for (const robot_state::RobotStatePtr &rs : _robot_state_array) {
    trajectory.addSuffixWayPoint(*rs, 0.0);
  }
  parabolic_parametrizer_.computeTimeStamps(trajectory, 0.5, 0.5);
  trajectory.getRobotTrajectoryMsg(_trj_msg);

  return true;
}
