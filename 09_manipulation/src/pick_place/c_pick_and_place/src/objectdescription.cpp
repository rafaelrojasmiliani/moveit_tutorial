#include "objectdescription.hpp"
#include <eigen_conversions/eigen_msg.h> // poseMsgToEigen
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <robot_state_publisher/robot_state_publisher.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_eigen/tf2_eigen.h> // eigenToTransform
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_broadcaster.h> // TransformBroadcaster
#include <urdf_model/link.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "urdf_to_collision_object.hpp"
#include <algorithm> // remove_if
#include <moveit/collision_detection_bullet/bullet_integration/ros_bullet_utils.h> // urdfPose2Eigen
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

ObjectDescription::ObjectDescription(const std::string &_name)
    : // begin initialization list
      collision_object_(), ee_name_(""), name_(_name), nh_(), nh_object_(_name),
      robot_model_loader_("robot_description"),
      robot_model_(robot_model_loader_.getModel()), ideal_grasp_pose_(), psi_(),
      visual_tools_(new moveit_visual_tools::MoveItVisualTools(
          robot_model_->getModelFrame(), "/rviz_visual_tools")),
      urdf_model_(), object_description_file_()
// end initialization list
{
  // 0. Declare vars
  std::size_t error = 0;      // error handling var
  XmlRpc::XmlRpcValue xmlval; // used to stor xmlrpc values
  XmlRpc::XmlRpcValue xmlel;  // used to stor xmlrpc element
  // --------------------------
  // --- 1. Read Parameters
  // --------------------------
  //

  error +=
      !rosparam_shortcuts::get("object_description", nh_object_,
                               "object_description", object_description_file_);

  if (not error) {
    error += !urdf_model_.initFile(object_description_file_);
  }
  if (not error) {
    urdf_model_.initFile(object_description_file_);
    error += !urdf_to_collision_object::urdf_to_collision_object(
        urdf_model_, collision_object_);

    Eigen::Isometry3d initial_pose;
    error += !rosparam_shortcuts::get("object_description", nh_object_,
                                      "initial_pose", initial_pose);

    read_sub_frames();
    urdf_to_collision_object::update_frames(initial_pose, collision_object_);
    collision_object_.id = _name;
    collision_object_.header.frame_id = "world";
    if (not psi_.applyCollisionObject(collision_object_))
      ROS_INFO("Cannot deploy collision object\n ");
  }

  read_gripper_translation(ideal_grasp_.pre_grasp_approach, "approach");
  read_gripper_translation(ideal_grasp_.post_grasp_retreat, "retreat");
  read_gripper_pose(gripper_closure_, "gripper_closure");
  read_gripper_pose(gripper_opening_, "gripper_opening");

  error += !rosparam_shortcuts::get("object_description", nh_object_,
                                    "ideal_grasp_pose", ideal_grasp_pose_);

  error += !rosparam_shortcuts::get("object_description", nh_object_,
                                    "tcp_to_eef_mount_transform",
                                    tcp_to_eef_mount_transform_);

  error += !rosparam_shortcuts::get("object_description", nh_object_,
                                    "end_effector_name", ee_name_);

  grasp_arm_tool_flange_pose_ = ideal_grasp_pose_ * tcp_to_eef_mount_transform_;

  ideal_grasp_.grasp_pose.header.frame_id = "myrobot/move_group/object";
  ideal_grasp_.grasp_pose.pose = tf2::toMsg(grasp_arm_tool_flange_pose_);

  visual_tools_->loadRobotStatePub("robot_grasp_pose", false);

  ROS_INFO_STREAM("ROS MOVITE IT PLANNING FRAME "
                  << robot_model_->getModelFrame() << "\n");
}

ObjectDescription::~ObjectDescription() {}

Eigen::Isometry3d ObjectDescription::get_pose() {
  Eigen::Isometry3d result;
  std::map<std::string, geometry_msgs::Pose> pose_pair_list =
      psi_.getObjectPoses({name_});
  if (pose_pair_list.find(name_) != pose_pair_list.end()) {
    tf::poseMsgToEigen(collision_object_.primitive_poses[0], result);
  } else {
    ROS_ERROR("Object not found in plannins scene");
  }
  return result;
}

void ObjectDescription::read_sub_frames() {
  XmlRpc::XmlRpcValue xmlval; // used to stor xmlrpc values

  if (not nh_object_.hasParam("subframes"))
    return;

  nh_object_.getParam("subframes", xmlval);
  if (not(xmlval.getType() == XmlRpc::XmlRpcValue::TypeArray))
    return;

  for (std::size_t i = 0; i < xmlval.size(); ++i) {
    if (xmlval[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      if (xmlval[i].hasMember("name") and xmlval[i].hasMember("pose") and
          xmlval[i]["pose"].getType() == XmlRpc::XmlRpcValue::TypeArray and
          xmlval[i]["pose"].size() == 6 and
          xmlval[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString) {

        std::vector<double> xyz_rpy_vector(7);

        for (std::size_t j = 0; j < 6; ++j) {
          if (xmlval[i]["pose"][j].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            xyz_rpy_vector[j] = static_cast<int>(xmlval[i]["pose"][j]);
          } else if (xmlval[i]["pose"][j].getType() ==
                     XmlRpc::XmlRpcValue::TypeDouble) {
            xyz_rpy_vector[j] = static_cast<double>(xmlval[i]["pose"][j]);
          }
          ROS_INFO(" piint %+14.7lf \n", xyz_rpy_vector[j]);
        }

        Eigen::Isometry3d relative_pose;
        geometry_msgs::Pose pose;
        rosparam_shortcuts::convertDoublesToEigen(
            "object_description", xyz_rpy_vector, relative_pose);
        pose = tf2::toMsg(relative_pose);

        collision_object_.subframe_poses.push_back(pose);

        collision_object_.subframe_names.push_back(xmlval[i]["name"]);
      }
    }
  }
}

bool ObjectDescription::read_gripper_translation(
    moveit_msgs::GripperTranslation &_gt, const std::string _param) {

  XmlRpc::XmlRpcValue xmlval; // used to stor xmlrpc values

  std::vector<double> direction(3);

  if (not nh_object_.hasParam(_param))
    return false;

  nh_object_.getParam(_param, xmlval);

  if (not(xmlval.getType() == XmlRpc::XmlRpcValue::TypeStruct))
    return false;

  if (xmlval.hasMember("direction") and
      xmlval["direction"].getType() == XmlRpc::XmlRpcValue::TypeArray and
      xmlval["direction"].size() == 3) {
    for (std::size_t i = 0; i < 3; i++) {
      if (xmlval["direction"][i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        direction[i] = (double)static_cast<int>(xmlval["direction"][i]);
      } else if (xmlval["direction"][i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble) {
        direction[i] = static_cast<double>(xmlval["direction"][i]);
      } else {
        ROS_ERROR("cannot read parameter\n");
        return false;
      }
    }
  }
  _gt.direction.vector.x = direction[0];
  _gt.direction.vector.y = direction[1];
  _gt.direction.vector.z = direction[2];
  if (xmlval.hasMember("desired_distance") and
      xmlval["desired_distance"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    _gt.desired_distance = static_cast<double>(xmlval["desired_distance"]);
  }
  if (xmlval.hasMember("minimum_distance") and
      xmlval["minimum_distance"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    _gt.min_distance = static_cast<double>(xmlval["minimum_distance"]);
  }

  _gt.direction.header.frame_id = "myrobot/move_group/object";

  return true;
}
bool ObjectDescription::read_gripper_pose(trajectory_msgs::JointTrajectory &_jt,
                                          const std::string _param) {

  XmlRpc::XmlRpcValue xmlval; // used to stor xmlrpc values

  if (not nh_object_.hasParam(_param))
    return false;

  nh_object_.getParam(_param, xmlval);

  if (not(xmlval.getType() == XmlRpc::XmlRpcValue::TypeArray))
    return false;

  _jt.joint_names.resize(xmlval.size());
  _jt.points.resize(1);
  _jt.points[0].positions.resize(xmlval.size());

  for (std::size_t i = 0; i < xmlval.size(); ++i) {
    if (xmlval[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      if (xmlval[i].hasMember("joint_name") and
          xmlval[i]["joint_name"].getType() ==
              XmlRpc::XmlRpcValue::TypeString and
          xmlval[i].hasMember("position") and
          xmlval[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        _jt.joint_names[i] = static_cast<std::string>(xmlval[i]["joint_name"]);
        _jt.points[0].positions[i] = static_cast<double>(xmlval[i]["position"]);
        _jt.points[0].time_from_start = ros::Duration(0.5);
      }
      return true;
    }
  }
  return true;
}

void ObjectDescription::show_grasp_pose(const std::string &_group_name) {

  robot_state::RobotState rs(robot_model_);
  geometry_msgs::Pose pose_wrt_robot_base;
  Eigen::Isometry3d actual_pose_wrt_robot = get_pose();

  Eigen::Isometry3d grasp_pose_wrt_to_robot =
      actual_pose_wrt_robot * grasp_arm_tool_flange_pose_;

  rs.setFromIK(robot_model_->getJointModelGroup(_group_name),
               tf2::toMsg(grasp_pose_wrt_to_robot));

  visual_tools_->publishRobotState(rs, rviz_visual_tools::RED);
}
