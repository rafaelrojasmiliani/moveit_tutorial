/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Mike Lautman*/

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));

  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  const moveit::core::JointModelGroup *joint_model_group =
      robot_state->getJointModelGroup("panda_arm");

  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(
          robot_model, node_handle, "planning_plugin", "request_adapters"));

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function
  // available from the `kinematic_constraints`_ package.
  //
  // .. _kinematic_constraints:
  //     http://docs.ros.org/noetic/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "panda_arm";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(
          "panda_link8", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so
  // that it does not modify the world representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* Now, call the pipeline and check whether planning was successful. */
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>(
          "/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(),
                                     joint_model_group);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  /* First, set the state in the planning scene to the final state of the last
   * plan */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)
                    ->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(
      joint_model_group,
      response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Now, setup a joint space goal
  moveit::core::RobotState goal_state(*robot_state);
  std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state,
                                                      joint_model_group);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Before planning, we will need a Read Only lock on the planning scene so
  // that it does not modify the world representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(),
                                     joint_model_group);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Using a Planning Request Adapter
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // A planning request adapter allows us to specify a series of operations that
  // should happen either before planning takes place or after the planning
  // has been done on the resultant path

  /* First, set the state in the planning scene to the final state of the last
   * plan */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)
                    ->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(
      joint_model_group,
      response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // Now, set one of the joints slightly outside its upper limit
  const moveit::core::JointModel *joint_model =
      joint_model_group->getJointModel("panda_joint3");
  const moveit::core::JointModel::Bounds &joint_bounds =
      joint_model->getVariableBounds();
  std::vector<double> tmp_values(1, 0.0);
  tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
  robot_state->setJointPositions(joint_model, tmp_values);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so
  // that it does not modify the world representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  /* Now you should see three planned trajectories in series*/
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(),
                                     joint_model_group);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to finish the demo");

  ROS_INFO("Done");
  return 0;
}
