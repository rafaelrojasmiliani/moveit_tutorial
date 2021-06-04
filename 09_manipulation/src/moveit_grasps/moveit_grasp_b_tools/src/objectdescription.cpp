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
      robot_model_(robot_model_loader_.getModel()), suction_data_(nullptr),
      score_weights_(new moveit_grasps::SuctionGraspScoreWeights()),
      score_weights_values_(7, 0.0), ideal_grasp_pose_(), psi_(),
      visual_tools_(new moveit_visual_tools::MoveItVisualTools(
          robot_model_->getModelFrame(), "/rviz_visual_tools")),
      grasp_generator_(nullptr), urdf_model_(), kdl_tree_(),
      object_description_file_(), vector_of_solid_primitives_(),
      root_link_name_()
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
    ROS_INFO("---------------------------------------\n");
    urdf_model_.initFile(object_description_file_);
    error += !urdf_to_collision_object::urdf_to_collision_object(
        urdf_model_, collision_object_);
    ROS_INFO("---------------------------------------++++++\n");
    Eigen::Isometry3d initial_pose;
    error += !rosparam_shortcuts::get("object_description", nh_object_,
                                      "initial_pose", initial_pose);

    read_sub_frames();
    urdf_to_collision_object::update_frames(initial_pose, collision_object_);
    collision_object_.id = _name;
    collision_object_.header.frame_id = "world";
    if (not psi_.applyCollisionObject(collision_object_))
      ROS_INFO("--------------------- AAAEEROROR\n ");
  }

  std::size_t i = 0;
  for (const std::string &par :
       {"tranlation_x_score", "tranlation_y_score", "tranlation_z_score",
        "rotation_x_score", "rotation_y_score", "rotation_z_score",
        "overhang_score"}) {
    if (nh_object_.hasParam(par)) { // existence
      nh_object_.getParam("object_files", xmlval);
      if (xmlval.getType() == XmlRpc::XmlRpcValue::TypeDouble or
          xmlval.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        score_weights_values_[i] = xmlval;
      }
    }
    i++;
  }
  score_weights_->orientation_x_score_weight_ = score_weights_values_[0];
  score_weights_->orientation_y_score_weight_ = score_weights_values_[1];
  score_weights_->orientation_z_score_weight_ = score_weights_values_[2];
  score_weights_->translation_x_score_weight_ = score_weights_values_[3];
  score_weights_->translation_y_score_weight_ = score_weights_values_[4];
  score_weights_->translation_z_score_weight_ = score_weights_values_[5];
  score_weights_->overhang_score_weight_ = score_weights_values_[6];

  error += !rosparam_shortcuts::get("object_description", nh_object_,
                                    "ideal_grasp_pose", ideal_grasp_pose_);
  error += !rosparam_shortcuts::get("object_description", nh_object_,
                                    "end_effector_name", ee_name_);

  // -------------------------------------------------------
  // --- 2. Initialize grasp data and load grasp paramaters
  // -------------------------------------------------------

  visual_tools_->loadMarkerPub();
  visual_tools_->loadRobotStatePub("/display_robot_state");
  visual_tools_->loadTrajectoryPub("/display_planned_path");
  visual_tools_->loadSharedRobotState();
  visual_tools_->enableBatchPublishing();

  suction_data_ = std::make_shared<moveit_grasps::SuctionGraspData>(
      nh_, ee_name_, robot_model_);

  error += !suction_data_->loadGraspData(nh_, ee_name_);

  grasp_generator_ = std::make_shared<moveit_grasps::SuctionGraspGenerator>(
      visual_tools_, true);

  ROS_INFO_STREAM("ideal pose \n" << ideal_grasp_pose_.matrix() << '\n');
  grasp_generator_->setGraspScoreWeights(score_weights_);
  grasp_generator_->setIdealTCPGraspPose(ideal_grasp_pose_);
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

bool ObjectDescription::compute_grasp_candidates(
    std::vector<moveit_grasps::GraspCandidatePtr> &_candidates) {
  Eigen::Isometry3d pose = get_pose();
  grasp_generator_->generateGrasps(pose, 0.1, 0.1, 0.1, suction_data_,
                                   _candidates);
  return true;
}
