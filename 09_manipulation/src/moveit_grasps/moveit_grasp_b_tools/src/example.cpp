
// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>

#include "objectdescription.hpp"
#include "simplemanipulator.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
// moveit::planning_interface::PlanningSceneInterface
//
moveit_msgs::CollisionObject
get_colision_object(const std::string &_header_frame,
                    const shape_msgs::SolidPrimitive &_shape, double _x,
                    double _y, double _z);

shape_msgs::SolidPrimitive get_primitive(const std::string &_type);

moveit_msgs::CollisionObject
get_colision_object(const std::string &_header_frame,
                    const shape_msgs::SolidPrimitive &_shape, double _x,
                    double _y, double _z);

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "grasp_filter_demo");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node_handle;

  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::CollisionObject object;
  object = get_colision_object("world", get_primitive("box"), 0.7, 0, 1.5);
  object.id = "object";
  psi.applyCollisionObject(object);

  ObjectDescription object_description("object");

  ros::waitForShutdown();
}
