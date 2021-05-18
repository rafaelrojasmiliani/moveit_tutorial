
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

bool pick(moveit::planning_interface::MoveGroupInterface &_mgi,
          moveit::planning_interface::PlanningSceneInterface &_psi,
          const std::string &_object_id);
bool place(moveit::planning_interface::MoveGroupInterface &_mgi,
           moveit::planning_interface::PlanningSceneInterface &_psi,
           const std::string &_object_id);

int main(int argc, char **argv) {
  // ---------------------------------------
  // 1. Ros node initialization
  // ---------------------------------------
  std::string planning_frame;
  std::string object_id;
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
  planning_frame = mgi.getPlanningFrame();
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
  moveit_msgs::CollisionObject cylinder = get_colision_object(
      planning_frame, shape_msgs::SolidPrimitive::CYLINDER, 0.7, 0, 1.5);
  object_id = cylinder.id;

  psi.applyCollisionObject(cylinder);
  mgi.setStartStateToCurrentState();

  geometry_msgs::Pose target_pose_1 = cylinder.primitive_poses[0];
  target_pose_1.position.y += -0.1;
  target_pose_1.position.z += -0.05;
  geometry_msgs::Pose target_pose_2 = cylinder.primitive_poses[0];
  target_pose_2.position.y += 0.1;
  target_pose_2.position.z += -0.05;
  int operation = 0;
  while (ros::ok()) {
    geometry_msgs::Pose target_pose;
    switch (opertion) {
    case 0:
      mgi.setPoseTarget(target_pose_1);
      break;
    case 1:
      mgi.attachObject(object_id);
      break;
    case 2:
      mgi.setPoseTarget(target_pose_1);
      break;
    }
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_error_code = mgi.plan(my_plan);
    if (moveit_error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      break;
    }
    mgi.execute(my_plan);
  }

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

bool pick(moveit::planning_interface::MoveGroupInterface &_mgi,
          moveit::planning_interface::PlanningSceneInterface &_psi,
          const std::string &_object_id) {

  std::map<std::string, geometry_msgs::Pose> poses_map;
  geometry_msgs::Pose pose;
  bool result = false;
  poses_map = _psi.getObjectPoses({_object_id});
  if (not poses_map.empty() and poses_map.find(_object_id) != poses_map.end()) {
    pose = poses_map[object_id];
    pose.position.z -= 0.05;
    result = plan_and_move(_mgi, pose);
  }
  if (result) {
    mgi.attachObject(cylinder.id);
    result = true;
  }

  return result;
}

bool place(moveit::planning_interface::MoveGroupInterface &_mgi,
           moveit::planning_interface::PlanningSceneInterface &_psi,
           const std::string &_object_id) {

  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects_map;
  objects_map = _psi.getAttachedObjects({_object_id});
  bool result = false;
  if (not objects_map.empty() and
      objects_map.find(_object_id) != objects_map.end()) {
  }
}

bool plan_and_move(moveit::planning_interface::MoveGroupInterface &_mgi,
                   const geometry_msgs::Pose &pose) {

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  _mgi.setPoseTarget(poses_map[object_id]);
  moveit_error_code = _mgi.plan(my_plan);
  if (moveit_error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    return false;
  }
  _mgi.execute(my_plan);
  return true;
}
