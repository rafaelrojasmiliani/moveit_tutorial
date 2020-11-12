
/* Author: Rafael A. Rojas*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

moveit_msgs::CollisionObject get_box(const std::string& _id,  const std::string& _reference_frame){

  moveit_msgs::CollisionObject result;
  result.header = _header_frame;
  result.id = _id;
  {//CollisionObject.privimtives (brackets only for readability)
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;
    result.primitives.push_back(primitive);
  }

  {// CollisionObject.primitive_poses (brackets only for readability
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    result.primitive_poses.push_back(pose);
  }
  result.operation = attached_object.object.ADD;

  return result;
}

void attach_box(moveit_msgs::PlanningScene& scene, const std::string& link, const std::vector<const std::string>& contat_links ){

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = link;
  attached_object.object.header.frame_id = link;
  attached_object.object.id = "box";

  // attached_object.primitive

  // attached_object.primitive_poses

  // attached_object.operation 

  attached_object.touch_links = contat_links;
}
int main(int argc, char** argv)
{

  ros::init(argc, argv, "planning_scene_msg_ros_api_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::ServiceClient planning_scene_msg_diff_client =
      node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");

  planning_scene_msg_diff_client.waitForExistence();


  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.world.collision_objects.push_back(attached_object.object);
  planning_scene_msg.is_diff = true;

  moveit_msgs::ApplyPlanningScene planning_scence_service_call;
  planning_scence_service_call.request.scene = planning_scene_msg;
  planning_scene_msg_diff_client.call(planning_scence_service_call);

  ros::shutdown();
  return 0;
}

