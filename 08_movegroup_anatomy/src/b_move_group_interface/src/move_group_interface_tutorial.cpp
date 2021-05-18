
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
#include <moveit/planning_scene/planning_scene.h>
// planning_scene::PlanningScene

moveit_msgs::CollisionObject
get_colision_object(const std::string &_header_frame,
                    const shape_msgs::SolidPrimitive &_shape, double _x,
                    double _y, double _z);

int main(int argc, char **argv) {
  // ---------------------------------------
  // 1. Ros node initialization
  // ---------------------------------------
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration(2).sleep();

  ros::ServiceClient get_ps_client_ =
      nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  get_ps_client_.waitForExistence();

  moveit_msgs::GetPlanningScene get_ps_srv_msg;

  // ---------------------------------------
  // 2. Instantate moveit stuff
  // ---------------------------------------
  // 2.1 Robot Model loader
  robot_model_loader::RobotModelLoader my_robot_model_loader(
      "robot_description");
  // 2.2 Robot Model
  const robot_model::RobotModelPtr &my_robot_model =
      my_robot_model_loader.getModel();
  // 2.3 Planning Scene
  planning_scene::PlanningScene workspace(my_robot_model);
  // 2.4 Planning Group name
  const std::string planning_group_name =
      my_robot_model->getJointModelGroupNames().back();
  // 2.5 MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface mgi(planning_group_name);
  // 2.6 PlanningSceneInterface
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit::planning_interface::MoveItErrorCode moveit_error_code;

  get_ps_client_.call(get_ps_srv_msg);
  workspace.setPlanningSceneMsg(get_ps_srv_msg.response.scene);

  robot_state::RobotStatePtr rs = mgi.getCurrentState();

  // ---------------------------------------
  // 3. Get custon info from move_group node
  // ---------------------------------------
  std::string planning_frame = mgi.getPlanningFrame();
  ROS_INFO_NAMED("tutorial", "Planning frame: %s",
                 mgi.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s",
                 mgi.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  ROS_INFO_NAMED("tutorial", "DOF of the joint group %u",
                 mgi.getVariableCount());
  ROS_INFO_NAMED("tutorial", "Planner Id = %s", mgi.getPlannerId().c_str());

  // ---------------------------------------
  // 4. Add a random object and move it
  // ---------------------------------------
  moveit_msgs::CollisionObject ro = get_colision_object(
      planning_frame, shape_msgs::SolidPrimitive::CYLINDER, 0.7, 0, 1.5);

  geometry_msgs::Pose target_pose = ro.primitive_poses[0];
  target_pose.position.z += -0.05;
  psi.applyCollisionObject(ro);
  // mgi.setStartState(*mgi.getCurrentState());
  mgi.setStartStateToCurrentState();
  mgi.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_error_code = mgi.plan(my_plan);
  if (moveit_error_code == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    mgi.execute(my_plan);
  } else {
    ROS_INFO_NAMED("tutorial", "plan failed");
    ros::shutdown();
    return 0;
  }

  mgi.attachObject(ro.id);
  target_pose.position.z += -0.1;
  mgi.setPoseTarget(target_pose);
  moveit_error_code = mgi.plan(my_plan);
  if (moveit_error_code == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    mgi.execute(my_plan);
  } else {
    ROS_INFO_NAMED("tutorial", "plan failed");
  }

  ros::shutdown();
  return 0;
}
