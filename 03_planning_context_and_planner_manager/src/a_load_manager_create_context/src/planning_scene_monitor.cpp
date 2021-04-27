/* Author: Rafael A. Rojas*/

#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ApplyPlanningScene.h>

planning_scene_monitor::PlanningSceneMonitorPtr psm;

bool apply_planning_scene(moveit_msgs::ApplyPlanningScene::Request &req,
                          moveit_msgs::ApplyPlanningScene::Response &res) {
  psm->updateFrameTransforms();
  res.success = psm->newPlanningSceneMessage(req.scene);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));

  psm.reset(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();
  psm->providePlanningSceneService();

  ros::ServiceServer service = node_handle.advertiseService(
      "apply_planning_scene", apply_planning_scene);

  ros::waitForShutdown();

  return 0;
}
