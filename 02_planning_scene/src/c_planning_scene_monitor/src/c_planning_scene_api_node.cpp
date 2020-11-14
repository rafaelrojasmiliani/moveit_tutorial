
/* Author: Rafael A. Rojas*/
#include <cstdlib>

#include <solids.h>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// MoveIt
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

void random_objects(ros::ServiceClient &srv);

// 1. Initializes the ros node
// 2. Initializes the service client for apply_planning_scene
// 3. Add or remove objectss randomly until the node is killed.
int main(int argc, char **argv) {

  ros::init(argc, argv, "planning_scene_msg_ros_api_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::ServiceClient planning_scene_msg_diff_client =
      node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>(
          "apply_planning_scene");

  ROS_INFO("waiting for service");
  planning_scene_msg_diff_client.waitForExistence();
  ROS_INFO("service found");

  random_objects(planning_scene_msg_diff_client);

  ros::shutdown();
  return 0;
}

/* add or removes random objects in the scene
 1. Createa random shape
 2. Initialize a collision object with the instantiated shape
 * */
void random_objects(ros::ServiceClient &srv) {
  shape_msgs::SolidPrimitive shape;
  moveit_msgs::CollisionObject object;
  std::vector<std::string> object_list;
  moveit_msgs::ApplyPlanningScene planning_scence_service_call;
  moveit_msgs::PlanningScene planning_scene_msg;
  bool message_ok = false;
  while (ros::ok()) {
    std::srand(std::time(0));
    const int add_or_remove = std::rand() % 2;
    if (add_or_remove == 0) {
      const int shape_type = std::rand() % 4;
      switch (shape_type) {
      case 0:
        shape = get_primitive("box");
        break;
      case 1:
        shape = get_primitive("sphere");
        break;
      case 2:
        shape = get_primitive("cylinder");
        break;
      case 3:
        shape = get_primitive("cone");
        break;
      }
      double x =
          (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 4.0 -
          2.0;
      double y =
          (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 4.0 -
          2.0;
      double z =
          (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 4.0 -
          2.0;
      object = get_colision_object("table", shape, x, y, z);
      object_list.push_back(object.id);
      object.operation = object.APPEND;
      ROS_INFO("Adding object");
      message_ok = true;
    } else {
      if (!object_list.empty()) {
        object.operation = object.REMOVE;
        size_t idx = std::rand() % object_list.size();
        object.id = object_list[idx];
        object_list.erase(object_list.begin() + idx);
        ROS_INFO("removing object %s", object.id.c_str());
        message_ok = true;
      } else {
        message_ok = false;
      }
    }

    if (message_ok) {
      planning_scene_msg.world.collision_objects.push_back(object);
      planning_scene_msg.is_diff = true;

      planning_scence_service_call.request.scene = planning_scene_msg;
      bool srv_success = srv.call(planning_scence_service_call);

      if (srv_success) {
        ROS_INFO("Service called successfuly");
      } else {
        ROS_INFO("Service failed");
      }

      ros::WallDuration(2.0).sleep();
    }
  }
}
