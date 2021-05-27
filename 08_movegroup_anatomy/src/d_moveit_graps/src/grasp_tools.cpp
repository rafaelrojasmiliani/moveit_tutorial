#include "grasp_tools.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_grasps/suction_grasp_filter.h>

moveit_grasps::SuctionGraspScoreWeightsPtr
get_grasp_suction_score(double _x, double _y, double _z, double _raw,
                        double _pich, double _yaw, double _overhang) {

  std::shared_ptr<moveit_grasps::SuctionGraspScoreWeights> result;

  result->orientation_x_score_weight_ = _raw;
  result->orientation_y_score_weight_ = _pich;
  result->orientation_z_score_weight_ = _yaw;
  result->translation_x_score_weight_ = _x;
  result->translation_y_score_weight_ = _y;
  result->translation_z_score_weight_ = _z;
  result->overhang_score_weight_ = _overhang;

  return result;
}

moveit_grasps::SuctionGraspDataPtr get_grasp_data_initialized_from_parameters(
    const ros::NodeHandle &_nh, const std::string &_end_effector_name,
    const moveit::core::RobotModelConstPtr &_robot_model) {

  moveit_grasps::SuctionGraspDataPtr result =
      std::make_shared<moveit_grasps::SuctionGraspData>(_nh, _end_effector_name,
                                                        _robot_model);

  result->loadGraspData(_nh, _end_effector_name);

  return result;
}

moveit_grasps::SuctionGraspGeneratorPtr get_initialized_grasp_generator(
    moveit_visual_tools::MoveItVisualToolsPtr _visual_tools,
    moveit_grasps::SuctionGraspScoreWeightsPtr _scores,
    std::vector<double> &_ideal_orientation) {

  moveit_grasps::SuctionGraspGeneratorPtr result =
      std::make_shared<moveit_grasps::SuctionGraspGenerator>(_visual_tools);

  result->setGraspScoreWeights(_scores);
  result->setIdealTCPGraspPoseRPY(_ideal_orientation);
  return result;
}

bool get_feasible_grasp_poses(
    moveit_visual_tools::MoveItVisualToolsPtr _visual_tools,
    std::vector<moveit_grasps::GraspCandidatePtr> &_grasp_candidates,
    const std::string &_arm_name) {

  // ---- Declration
  moveit_grasps::SuctionGraspFilterPtr grasp_filter;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_state::RobotStatePtr rs;
  const moveit::core::JointModelGroup *arm_jmg; // THIS may be const!

  // -- initialize the planning scene monitor and the robot state
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          "robot_description");

  planning_scene_monitor->requestPlanningSceneState();

  rs = std::make_shared<robot_state::RobotState>(
      planning_scene_monitor->getPlanningScene()->getCurrentState());

  const robot_model::RobotModelConstPtr &robot_model =
      planning_scene_monitor->getRobotModel();
  arm_jmg = robot_model->getJointModelGroup(_arm_name);
  // Initialize the grasp filter
  grasp_filter =
      std::make_shared<moveit_grasps::SuctionGraspFilter>(rs, _visual_tools);

  bool filter_pregrasps = true;
  grasp_filter->setSuctionVoxelOverlapCutoff(0.5);
  if (!grasp_filter->filterGrasps(_grasp_candidates, planning_scene_monitor,
                                  arm_jmg, seed_state, filter_pregrasps,
                                  object_name)) {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Filter grasps failed");
    return false;
  }
}

bool get_initial_pose(Eigen::Isometry3d _object_pose,
                      moveit_grasps::SuctionGraspDataPtr _data) {

  Eigen::Isometry3d position_of_the_ee_moint_flange =
      _object_pose * _data->tcp_to_eef_mount_.inverse();
}
