/* Author: Rafael A. Rojas*/
/*ros::init
  ros::AsyncSpinner
 * */
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model/robot_model.h> //robot_model::RobotModel
#include <moveit/robot_model_loader/robot_model_loader.h> // robot_model_loader::RobotModelLoader
#include <moveit/robot_state/robot_state.h>

class ShowRobotState {
public:
  ShowRobotState()
      : loader_("robot_description"), model_(loader_.getModel()),
        workspace_(new planning_scene::PlanningScene(model_)),
        visual_tool_("table") {
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers(); // clear all old markers
    visual_tools.trigger()
  }
  virtual ~ShowRobotState() {}
  void spin() {
    while (ros::ok()) {
      ros::spinOnce();
    }
  }
  robot_model_loader::RobotModelLoader loader_;
  const robot_model::RobotModelPtr model_;
  planning_scene::PlanningScenePtr workspace_;
  moveit_visual_tools::MoveItVisualTools visual_tool_;
};

int main(int argc, char **argv) {
  // Set up ROS.
  ros::init(argc, argv, "a_show_robot_state_node");
  ros::NodeHandle nh;

  // ros::shutdown();
  // ros::waitForShutdown();
  return 0;
}
