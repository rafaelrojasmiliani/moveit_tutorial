/* Author: Rafael A. Rojas*/
/*ros::init
  ros::AsyncSpinner
 * */
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model/robot_model.h> //robot_model::RobotModel
#include <moveit/robot_model_loader/robot_model_loader.h> // robot_model_loader::RobotModelLoader
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h> //planning_scene::PlanningScene
#include <moveit_visual_tools/moveit_visual_tools.h>

class ShowRobotState {
public:
  ShowRobotState()
      : loader_("robot_description"), model_(loader_.getModel()),
        workspace_(model_), visualizer01_("table"), visualizer02_("table") {
    // instantiate a  a publisher of moveit_msgs::DisplayRobotState to the
    // /display_robot_state topic
    visualizer01_.loadRobotStatePub("display_robot_state_1");
    visualizer02_.loadRobotStatePub("display_robot_state_2");
  }

  ~ShowRobotState() {}

  void spin() {
    robot_state::RobotState &state = workspace_.getCurrentStateNonConst();
    const robot_model::JointModelGroup *joint_group =
        model_->getJointModelGroup(model_->getJointModelGroupNames().back());
    visualizer01_.setAlpha(0.1);
    visualizer02_.setAlpha(0.5);
    while (ros::ok()) {
      state.setToRandomPositions(joint_group);
      visualizer01_.publishRobotState(state, rviz_visual_tools::GREEN);
      state.setToRandomPositions(joint_group);
      visualizer02_.publishRobotState(state, rviz_visual_tools::RED);
      // visualizer01_.trigger();
      ros::WallDuration(1).sleep();
      ros::spinOnce();
    }
  }
  robot_model_loader::RobotModelLoader loader_;
  const robot_model::RobotModelPtr model_;
  planning_scene::PlanningScene workspace_;
  moveit_visual_tools::MoveItVisualTools visualizer01_;
  moveit_visual_tools::MoveItVisualTools visualizer02_;
};

int main(int argc, char **argv) {
  // Set up ROS.
  ros::init(argc, argv, "a_show_robot_state_node");
  ros::NodeHandle nh;

  ShowRobotState srs;

  srs.spin();

  return 0;
}
