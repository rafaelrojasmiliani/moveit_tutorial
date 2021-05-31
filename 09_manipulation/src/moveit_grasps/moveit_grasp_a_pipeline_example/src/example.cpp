
// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "grasp_tools.h"

#include <moveit/move_group_interface/move_group_interface.h>
// moveit::planning_interface::MoveGroupInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// moveit::planning_interface::PlanningSceneInterface
//
moveit_msgs::CollisionObject
get_colision_object(const std::string &_header_frame,
                    const shape_msgs::SolidPrimitive &_shape, double _x,
                    double _y, double _z);

shape_msgs::SolidPrimitive get_primitive(const std::string &_type);
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
  // 3. deploy object
  // ---------------------------

  moveit_msgs::CollisionObject object;
  object = get_colision_object(mgi.getPlanningFrame(), get_primitive("box"),
                               0.7, 0, 1.5);
  psi.applyCollisionObject(object);
  ros::Duration(3).sleep(); // wait for movegroup to process call
  // -------------------
  // 4. Declrare Moveit Grasp objects
  // ---------------------------
  moveit_grasps::GraspPlannerPtr grasp_planner;
  moveit_grasps::SuctionGraspScoreWeightsPtr grasp_scores;
  moveit_grasps::SuctionGraspDataPtr grasp_data;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  moveit::planning_interface::MoveItErrorCode moveit_error;
  std::vector<moveit::core::RobotStatePtr> robot_state_sequence;
  // moveit_grasps::SuctionGraspGeneratorPtr grasp_generator;
  // -------------------
  // 5. Get Grasp data from the ROS context (context=parameters in namespace)
  // ---------------------------

  grasp_data = get_grasp_data_initialized_from_parameters(
      node_handle, "end_effector", robot_model);
  // ------------------------
  // 6. Get grasping candidates
  // ------------------------
  // 6.1 Get grasping scores
  grasp_scores = get_grasp_suction_score(1, 1, 1, 2, 2, 2, 10);
  // 6.2 Compute ideal grasp pose
  Eigen::Isometry3d ideal_grasp_pose;
  ideal_grasp_pose.setIdentity();
  // 6.3 get a vector of feasible grasp candidates for the given object
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
  get_feasible_grasp_poses(grasp_candidates, "arm", object.id, grasp_data,
                           grasp_scores, ideal_grasp_pose);

  // ------------------------
  // 6. Get grasping candidates
  // ------------------------

  grasp_planner = plan_grasp(grasp_candidates[0], object.id);

  robot_state::RobotStatePtr pre_grasp_state =
      grasp_candidates[0]
          ->segmented_cartesian_traj_[moveit_grasps::APPROACH]
          .front();
  mgi.setJointValueTarget(*pre_grasp_state);
  moveit_error = mgi.plan(motion_plan);
  if (moveit_error == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    mgi.execute(motion_plan);

    mgi.setStartStateToCurrentState();

    trajectory_processing::IterativeParabolicTimeParameterization tptp;

    moveit_msgs::RobotTrajectory trajectory_message;

    robot_trajectory::RobotTrajectory trajectory(robot_model, "arm");
    std::vector<robot_state::RobotStatePtr> &approach =
        grasp_candidates[0]->segmented_cartesian_traj_[moveit_grasps::APPROACH];
    for (const robot_state::RobotStatePtr &rs : approach) {
      trajectory.addSuffixWayPoint(*rs, 0.0);
    }
    tptp.computeTimeStamps(trajectory, 0.1, 0.1);
    trajectory.getRobotTrajectoryMsg(trajectory_message);

    mgi.execute(trajectory_message);
  }

  ros::shutdown();
}
