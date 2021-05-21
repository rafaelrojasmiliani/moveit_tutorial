
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
#include <moveit_msgs/PickupGoal.h>

moveit_msgs::CollisionObject
get_colision_object(const std::string &_header_frame,
                    const shape_msgs::SolidPrimitive &_shape, double _x,
                    double _y, double _z);

shape_msgs::SolidPrimitive get_primitive(const std::string &_type);
geometry_msgs::PoseStamped
compute_arm_flange_pose(const moveit_msgs::CollisionObject &_object);

moveit_msgs::GripperTranslation
compute_how_to_approach_object(const moveit_msgs::CollisionObject &_object);

moveit_msgs::GripperTranslation
compute_how_to_extract_object(const moveit_msgs::CollisionObject &_object);

int main(int argc, char **argv) {
  // ---------------------------------------
  // 1. Ros node initialization
  // ---------------------------------------
  std::string planning_frame;
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration(5).sleep();

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
  planning_frame = mgi.getPlanningFrame();
  // 2.6 PlanningSceneInterface
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit::planning_interface::MoveItErrorCode moveit_error_code;

  get_ps_client_.call(get_ps_srv_msg);
  workspace.setPlanningSceneMsg(get_ps_srv_msg.response.scene);

  // 3. Insert the collision object into the planning scene
  moveit_msgs::CollisionObject object;
  object =
      get_colision_object(planning_frame, get_primitive("box"), 0.7, 0, 1.5);
  psi.applyCollisionObject(object);

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  ROS_INFO("------------------------------------");
  grasps[0].grasp_pose = compute_arm_flange_pose(object);
  grasps[0].pre_grasp_approach = compute_how_to_approach_object(object);
  grasps[0].post_grasp_retreat = compute_how_to_extract_object(object);

  ROS_INFO("------------------------------------");
  ROS_INFO("------------------------------------");
  ROS_INFO("------------------------------------");
  mgi.pick(object.id, grasps);

  ros::shutdown();
  return 0;
}

geometry_msgs::PoseStamped
compute_arm_flange_pose(const moveit_msgs::CollisionObject &_object) {

  // this computes the position of the link of the planning group
  // that is father of the wripper.

  double box_size = _object.primitives[0].dimensions[2];
  geometry_msgs::PoseStamped result;

  moveit::planning_interface::PlanningSceneInterface psi;

  // parallel to the object frame and bellow the object
  result.header.frame_id = _object.header.frame_id;
  result.pose = _object.primitive_poses[0];
  result.pose.position.z -= (0.02 + box_size / 2);

  return result;
}

moveit_msgs::GripperTranslation
compute_how_to_approach_object(const moveit_msgs::CollisionObject &_object) {
  moveit_msgs::GripperTranslation result;
  result.direction.vector.z = 1;
  result.direction.header.frame_id = _object.id;
  result.min_distance = 0.05;
  result.desired_distance = 0.06;
  return result;
}

moveit_msgs::GripperTranslation
compute_how_to_extract_object(const moveit_msgs::CollisionObject &_object) {
  moveit_msgs::GripperTranslation result;
  result.direction.vector.z = -1;
  result.direction.header.frame_id = _object.id;
  result.min_distance = 0.05;
  result.desired_distance = 0.06;
  return result;
}
