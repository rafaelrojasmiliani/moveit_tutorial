

#include <moveit/move_group_interface/move_group_interface.h>
// moveit::planning_interface::MoveGroupInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// moveit::planning_interface::PlanningSceneInterface
#include <moveit/robot_model/robot_model.h> //robot_model::RobotModel
#include <moveit/robot_model_loader/robot_model_loader.h>
// robot_model_loader::RobotModelLoader
#include <moveit/robot_state/robot_state.h> // robot_state::RobotStatePtr
#include <moveit_msgs/GetPlanningScene.h>
// GetPlanningScene service
#include <moveit_visual_tools/moveit_visual_tools.h>

// Grasp
#include <moveit_grasps/two_finger_grasp_generator.h>
#include <moveit_grasps/two_finger_grasp_data.h>
#include <moveit_grasps/two_finger_grasp_filter.h>
#include <moveit_grasps/grasp_planner.h>

moveit_msgs::CollisionObject
get_colision_object(const std::string &_header_frame,
                    const shape_msgs::SolidPrimitive &_shape, double _x,
                    double _y, double _z);

shape_msgs::SolidPrimitive get_primitive(const std::string &_type);

int main(int argc, char **argv) {
  // ---------------------------------------
  // 1. Ros node initialization
  // ---------------------------------------
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration(2).sleep();


  moveit_msgs::GetPlanningScene get_ps_srv_msg;

  // ---------------------------------------
  // 2. Instantate moveit stuff
  // ---------------------------------------
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  // 2.1 Robot Model loader
  robot_model_loader::RobotModelLoader my_robot_model_loader(
      "robot_description");
  // 2.2 Robot Model
  const robot_model::RobotModelPtr &my_robot_model =
      my_robot_model_loader.getModel();
  // 2.3 Planning Group name
  const std::string planning_group_name =
      my_robot_model->getJointModelGroupNames().back();
  // 2.4 MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface mgi(planning_group_name);
  // 2.5 PlanningSceneInterface
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit::planning_interface::MoveItErrorCode moveit_error_code;


  robot_state::RobotStatePtr rs = mgi.getCurrentState();

  // ---------------------------------------
  // 3. Get custon info from move_group node
  // ---------------------------------------
  std::string planning_frame = mgi.getPlanningFrame();
  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(planning_frame);

  // ---------------------------------------
  // 4. Add a random object and move it
  // ---------------------------------------
  moveit_msgs::CollisionObject object;
  object =
      get_colision_object(planning_frame, get_primitive("box"), 0.7, 0, 1.5);

  moveit_grasps::TwoFingerGraspDataPtr grasp_data;
  moveit_grasps::TwoFingerGraspGeneratorPtr grasp_generator;

  grasp_data = std::make_shared<moveit_grasps::TwoFingerGraspData>(nh, "end_effector",my_robot_model);

  grasp_generator = std::make_shared<moveit_grasps::TwoFingerGraspGenerator>(visual_tools);

  ros::shutdown();
  return 0;
}
