#include "objectdescription.hpp"
#include <eigen_conversions/eigen_msg.h> // poseMsgToEigen
#include <memory>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_eigen/tf2_eigen.h>           // eigenToTransform
#include <tf2_ros/transform_broadcaster.h> // TransformBroadcaster
#include <xmlrpcpp/XmlRpcValue.h>

ObjectDescription::ObjectDescription(const std::string &_name)
    : collision_object_(), ee_name_(""), name_(_name), nh_(),
      robot_model_loader_("robot_description"),
      robot_model_(robot_model_loader_.getModel()), suction_data_(nullptr),
      score_weights_(new moveit_grasps::SuctionGraspScoreWeights()),
      score_weights_values_(7, 0.0), ideal_grasp_pose_(), psi_(),
      keep_running_(true), thread_(),
      visual_tools_(new moveit_visual_tools::MoveItVisualTools(
          robot_model_->getModelFrame(), "/rviz_visual_tools")),
      grasp_generator_(nullptr), grasp_candidates_() {
  // --------------------------
  // --- 1. Get object from scene
  // --------------------------
  std::size_t error = 0;

  std::map<std::string, moveit_msgs::CollisionObject> object_pair_list =
      psi_.getObjects({_name});
  if (object_pair_list.find(_name) == object_pair_list.end()) {
    ROS_ERROR("Object not found in plannins scene");
    error += 1;
  } else {
    collision_object_ = object_pair_list[_name];
  }
  // --------------------------
  // --- 2. Read Parameters
  // --------------------------
  XmlRpc::XmlRpcValue xmlval;

  ros::NodeHandle object_nh(nh_, _name);
  std::size_t i = 0;
  for (const std::string &par :
       {"tranlation_x_score", "tranlation_y_score", "tranlation_z_score",
        "rotation_x_score", "rotation_y_score", "rotation_z_score",
        "overhang_score"}) {
    if (object_nh.hasParam(par)) { // existence
      object_nh.getParam("object_files", xmlval);
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

  error += !rosparam_shortcuts::get("object_description", object_nh,
                                    "ideal_grasp_pose", ideal_grasp_pose_);
  error += !rosparam_shortcuts::get("object_description", object_nh,
                                    "end_effector_name", ee_name_);

  // -------------------------------------------------------
  // --- 3. Initialize grasp data and load grasp paramaters
  // -------------------------------------------------------

  suction_data_ = std::make_shared<moveit_grasps::SuctionGraspData>(
      nh_, ee_name_, robot_model_);

  error += !suction_data_->loadGraspData(nh_, ee_name_);
  /*
    grasp_generator_ = std::make_shared<moveit_grasps::SuctionGraspGenerator>(
        nh_, ee_name_, robot_model_);
    grasp_generator_->setGraspScoreWeights(score_weights_);
    grasp_generator_->setIdealTCPGraspPose(ideal_grasp_pose_);
    grasp_generator_->generateGrasps(object_pose, object_x_depth,
    object_y_width, object_z_height, suction_data_, _grasp_candidates);
    */

  if (not error) {
    thread_ = std::thread(&ObjectDescription::publishing_thread, this);
  }
}

ObjectDescription::~ObjectDescription() {
  keep_running_ = false;
  thread_.join();
}
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

void ObjectDescription::publishing_thread() {
  geometry_msgs::TransformStamped transform;
  tf2_ros::TransformBroadcaster broadcaster;
  ros::Rate rate(10.0);
  while (keep_running_) {
    transform = tf2::eigenToTransform(ideal_grasp_pose_);
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = name_;
    transform.child_frame_id = name_ + "ideal_grasp_pose";
    broadcaster.sendTransform(transform);

    transform = tf2::eigenToTransform(ideal_grasp_pose_ *
                                      suction_data_->tcp_to_eef_mount_);
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = name_;
    transform.child_frame_id = name_ + "ideal_arm_ee_flange_pose";
    broadcaster.sendTransform(transform);

    transform = tf2::eigenToTransform(ideal_grasp_pose_ *
                                      suction_data_->tcp_to_eef_mount_);
    rate.sleep();
  }
}
