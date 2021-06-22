
// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>

#include "objectdescription.hpp"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
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

  tf2_ros::StaticTransformBroadcaster stb;
  geometry_msgs::TransformStamped st;

  moveit::planning_interface::MoveGroupInterface mgi("arm");
  moveit::planning_interface::MoveGroupInterface mgi_gripper("end_effector");

  ObjectDescription object_description("object");
  ros::Duration(1).sleep();

  object_description.show_grasp_pose("arm");

  st.transform.translation.x =
      object_description.ideal_grasp_.grasp_pose.pose.position.x;
  st.transform.translation.y =
      object_description.ideal_grasp_.grasp_pose.pose.position.y;
  st.transform.translation.z =
      object_description.ideal_grasp_.grasp_pose.pose.position.z;
  st.transform.rotation =
      object_description.ideal_grasp_.grasp_pose.pose.orientation;

  st.child_frame_id = "ideal_flage_pose";

  st.header = object_description.ideal_grasp_.grasp_pose.header;

  st.header.stamp = ros::Time::now();
  stb.sendTransform(st);
  tf::transformEigenToMsg(object_description.ideal_grasp_pose_, st.transform);
  st.child_frame_id = "ideal_tcp_pose";
  stb.sendTransform(st);

  ROS_INFO("planner ID = %s \n", mgi.getDefaultPlannerId("arm").c_str());
  mgi.setPlannerParams(
      "RRT", "arm", {std::make_pair("goal_bias", "0.8"),
                     std::make_pair("longest_valid_segment_fraction", "0.1")});
  mgi_gripper.setJointValueTarget({0.0});
  mgi_gripper.move();
  mgi.pick(object_description.get_name(), {object_description.ideal_grasp_});

  mgi.detachObject(object_description.get_name());

  ros::waitForShutdown();
}
