

#include<shape_msgs/SolidPrimitive.h>
#include<moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <string>

shape_msgs::SolidPrimitive get_primitive(const std::string &_type) {

  shape_msgs::SolidPrimitive result;

  if (_type == "box") {
    result.type = result.BOX;
    result.dimensions.resize(3);
    result.dimensions[0] = 0.1;
    result.dimensions[1] = 0.1;
    result.dimensions[2] = 0.1;
  } else if (_type == "sphere") {
    result.type = result.SPHERE;
    result.dimensions.resize(1);
    result.dimensions[0] = 0.1;
  } else if (_type == "cylinder") {
    result.type = result.CYLINDER;
    result.dimensions.resize(2);
    result.dimensions[0] = 0.1;
    result.dimensions[1] = 0.1;
  } else if (_type == "cone") {
    result.type = result.CONE;
    result.dimensions.resize(2);
    result.dimensions[0] = 0.1;
    result.dimensions[1] = 0.1;
  }

  return result;
}

moveit_msgs::CollisionObject
get_colision_object(const std::string &_header_frame,
                    const shape_msgs::SolidPrimitive &_shape, double _x,
                    double _y, double _z) {
  static unsigned int counter = 0;
  std::string object_name = "object" + std::to_string(counter);
  counter++;

  moveit_msgs::CollisionObject result;

  result.header.frame_id = _header_frame;
  result.id = object_name;

  result.primitives.push_back(_shape);

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = _x;
  pose.position.y = _y;
  pose.position.z = _z;
  result.primitive_poses.push_back(pose);

  return result;
}


/* add or removes random objects in the scene
 1. Create/remove random objects into the planning scene
 2. Call the motion planning service
 * */
void random_objects(ros::ServiceClient &_srv_planning_scene, ros::ServiceClient &_srv_planning) {
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
          (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 3.0 -
          1.5;
      double y =
          (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 3.0 -
          1.5;
      double z =
          (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 3.0 -
          1.5;
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
      bool srv_success = _srv_planning_scene.call(planning_scence_service_call);

      if (srv_success) {
        ROS_INFO("Service called successfuly");
        std_srvs::Trigger trigger_msg;
        _srv_planning.call(trigger_msg);
      } else {
        ROS_INFO("Service failed");
      }

      ros::WallDuration(2.0).sleep();
    }
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "orchestrator");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

// planning scene service
  ros::ServiceClient planning_scene_msg_diff_client =
      node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>(
          "apply_planning_scene");
// motion planning serice
  ros::ServiceClient plan_random_motion_client =
      node_handle.serviceClient<std_srvs::Trigger>(
          "plan_random_motion");
  ROS_INFO("waiting for service");
  planning_scene_msg_diff_client.waitForExistence();
  plan_random_motion_client.waitForExistence();
  ROS_INFO("services found");

  random_objects(planning_scene_msg_diff_client, plan_random_motion_client);

  ros::shutdown();
  return 0;
}
