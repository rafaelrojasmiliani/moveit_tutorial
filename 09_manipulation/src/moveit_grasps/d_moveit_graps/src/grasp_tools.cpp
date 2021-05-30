#include "grasp_tools.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_grasps/suction_grasp_filter.h>
#include<ros/ros.h>
namespace {
bool isStateValid(const planning_scene::PlanningScene *planning_scene,
                  robot_state::RobotState *robot_state,
                  const robot_model::JointModelGroup *group,
                  const double *ik_solution) {
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();
  bool colides =planning_scene->isStateColliding(*robot_state, group->getName(), true); 
  if(colides){
      ROS_INFO("is state valied: COLIDES\n");
  }
  return !colides;
}

} // namespace
moveit_grasps::SuctionGraspScoreWeightsPtr
get_grasp_suction_score(double _x, double _y, double _z, double _raw,
                        double _pich, double _yaw, double _overhang) {
  std::shared_ptr<moveit_grasps::SuctionGraspScoreWeights> result;
  result = std::make_shared<moveit_grasps::SuctionGraspScoreWeights>();
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
    Eigen::Isometry3d &_ideal_grasp_pose) {

  moveit_grasps::SuctionGraspGeneratorPtr result =
      std::make_shared<moveit_grasps::SuctionGraspGenerator>(_visual_tools, true);

  ROS_INFO("%s-----------\n", result->getVerbose()?"verbose":"No Verbose");

  result->setGraspScoreWeights(_scores);
  result->setIdealTCPGraspPose(_ideal_grasp_pose);
  return result;
}

robot_state::RobotStatePtr
get_initial_robot_state(const std::string _arm_name,const std::string _ee_name, Eigen::Isometry3d _object_pose,
                        Eigen::Isometry3d _tcp_to_eef_mount) {

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  const moveit::core::JointModelGroup *arm_jmg; // THIS may be const!
  robot_state::RobotStatePtr result;

  // initialize plannins scene monitor
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          "robot_description");

  planning_scene_monitor->requestPlanningSceneState();

  // Get the current robot state
  result = std::make_shared<robot_state::RobotState>(
      planning_scene_monitor->getPlanningScene()->getCurrentState());

  // Get the robot model and the joint group
  const robot_model::RobotModelConstPtr &robot_model =
      planning_scene_monitor->getRobotModel();

  arm_jmg = robot_model->getJointModelGroup(_arm_name);

  const moveit::core::JointModelGroup* ee_jmg =
      robot_model->getEndEffector(_ee_name);

  const std::string arm_flange_link_name = ee_jmg->getEndEffectorParentGroup().second;

      // ---- actual work
      // This is an arbitrary pose, used only as a seed for the grasp filter
      Eigen::Isometry3d arm_flange_target_pose =
          _object_pose * _tcp_to_eef_mount;
  //  _data->tcp_to_eef_mount_.inverse();

  moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
      &isStateValid, planning_scene_monitor->getPlanningScene().get(), _1, _2, _3);
   
  if (result->setFromIK(arm_jmg, arm_flange_target_pose, arm_flange_link_name, 10.0, constraint_fn))
    return result;
  return result;
}

bool get_feasible_grasp_poses(
    std::vector<moveit_grasps::GraspCandidatePtr> &_grasp_candidates,
    const std::string &_arm_name, const std::string &_object_name,
    moveit_grasps::SuctionGraspDataPtr _grasp_data,
    moveit_grasps::SuctionGraspScoreWeightsPtr _scores,
    Eigen::Isometry3d &_ideal_grasp_pose) {

  // ------------------------
  // 1. Declaration of variables
  // ------------------------
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_state::RobotStatePtr robot_state;
  const moveit::core::JointModelGroup *arm_jmg; // THIS may be const!
  moveit_msgs::CollisionObject collision_obj;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  geometry_msgs::Pose object_pose_msg;
  Eigen::Isometry3d object_pose;
  //       Declear grasp stuff
  moveit_grasps::SuctionGraspFilterPtr grasp_filter;

    moveit_grasps::SuctionGraspGeneratorPtr grasp_generator;

  // -------------------------------
  // 2. Get data from Planning scene
  // -------------------------------
  // 2.1 Initialize the planning scene monitor fomr the ROS context
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          "robot_description");

  // 2.2 get the plannins scene from the movegroup
  planning_scene_monitor->requestPlanningSceneState();

  // 2.2 Get the obstacle
  ROS_INFO("----------- calling tf\n");
  planning_scene_monitor->getPlanningScene()->getCollisionObjectMsg(
      collision_obj, _object_name);
  
  ROS_INFO("----------- object name =  %s\n", _object_name.c_str());
  ROS_INFO("----------- object name =  %s\n", collision_obj.id.c_str());

  ROS_INFO("----------- calling tf\n");
  object_pose_msg = collision_obj.primitive_poses[0];
  double object_x_depth = collision_obj.primitives[0].dimensions[0];
  double object_y_width = collision_obj.primitives[0].dimensions[1];
  double object_z_height = collision_obj.primitives[0].dimensions[2];

  ROS_INFO("----------- calling tf\n");
  tf::poseMsgToEigen(object_pose_msg, object_pose);
  ROS_INFO("-----------\n");


  ROS_INFO("PLANNINS SCENE MONITOR AND OBJECT DONE\n");
  // -------------------------------
  // 3. Compute the robot configuration unsed as seed by the filter
  // -------------------------------
  robot_state =
      get_initial_robot_state("arm", "end_effector",object_pose, _grasp_data->tcp_to_eef_mount_);

  if(!robot_state){
  ROS_INFO("We failed to find a feasible position\n");
  return false;
  }

  ROS_INFO("ROBOT SEED COMPUTED\n");
  const robot_model::RobotModelConstPtr &robot_model =
      planning_scene_monitor->getRobotModel();

  arm_jmg = robot_model->getJointModelGroup(_arm_name);

  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      robot_model->getModelFrame(), "/rviz_visual_tools");// ---
    visual_tools->loadMarkerPub();
    visual_tools->loadRobotStatePub("/display_robot_state");
    visual_tools->loadTrajectoryPub("/display_planned_path");
    visual_tools->loadSharedRobotState();
    visual_tools->enableBatchPublishing();

  grasp_generator = get_initialized_grasp_generator(visual_tools, _scores,_ideal_grasp_pose);

  ROS_INFO("grasp genereator done\n");
  if (!grasp_generator->generateGrasps(object_pose, object_x_depth,
                                        object_y_width, object_z_height,
                                        _grasp_data, _grasp_candidates)) {
        ROS_ERROR("Grasp generator failed to generate any valid grasps");
    return false;
  }
  ROS_INFO("------------ GRASP generated ----------------- \n");
  // Initialize the grasp filter
  grasp_filter = std::make_shared<moveit_grasps::SuctionGraspFilter>(
      robot_state, visual_tools);

  bool filter_pregrasps = true;
  grasp_filter->setSuctionVoxelOverlapCutoff(0.5);
  ROS_INFO("------------ calling grasp filter ----------------- \n");
  if (!grasp_filter->filterGrasps(_grasp_candidates, planning_scene_monitor,
                                  arm_jmg, robot_state, filter_pregrasps,
                                  _object_name)) {
    ROS_ERROR_STREAM("Filter grasps failed");
    return false;
  }
  return true;
}

moveit_grasps::GraspPlannerPtr
plan_grasp(
    moveit_grasps::GraspCandidatePtr &_grasp_candidate, const std::string &_object_name) {

  // ------------------------
  // 1. Declaration of variables
  // ------------------------
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_state::RobotStatePtr robot_state;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  moveit_grasps::GraspPlannerPtr grasp_planner;

  // -------------------------------
  // 2. Get data from Planning scene
  // -------------------------------
  // 2.1 Initialize the planning scene monitor fomr the ROS context
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          "robot_description");

  // 2.2 get the plannins scene from the movegroup
  planning_scene_monitor->requestPlanningSceneState();

  // 2.3 Get current robot state
  robot_state= std::make_shared<robot_state::RobotState>(
      planning_scene_monitor->getPlanningScene()->getCurrentState());
  // 2.4 Get robot model
  const robot_model::RobotModelConstPtr &robot_model =
      planning_scene_monitor->getRobotModel();
  // -------------------------------
  // 3. Initialize visual tools
  // -------------------------------
  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      robot_model->getModelFrame(), "/rviz_visual_tools");// ---
    visual_tools->loadMarkerPub();
    visual_tools->loadRobotStatePub("/display_robot_state");
    visual_tools->loadTrajectoryPub("/display_planned_path");
    visual_tools->loadSharedRobotState();
    visual_tools->enableBatchPublishing();


    grasp_planner = std::make_shared<moveit_grasps::GraspPlanner>(visual_tools);

    bool feasible_plan = grasp_planner->planApproachLiftRetreat(_grasp_candidate, robot_state, planning_scene_monitor, false,
                                                   _object_name);
    if(not feasible_plan)
        return nullptr;
  return grasp_planner;
}


void robot_state_vector_to_robot_trajectory(){
}
