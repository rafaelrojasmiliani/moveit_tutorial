#include "objectdescription.hpp"
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <xmlrpcpp/XmlRpcValue.h>

ObjectDescription::ObjectDescription(
    moveit_msgs::CollisionObject &_collision_object,
    const std::string &_ee_name, const std::string &_ns)
    : collision_object_(_collision_object), ee_name_(_ee_name), nh_(_ns),
      robot_model_loader_("robot_description"),
      robot_model_(robot_model_loader_.getModel()),
      suction_data_(
          new moveit_grasps::SuctionGraspData(nh_, _ee_name, robot_model_)),
      score_weights_(new moveit_grasps::SuctionGraspScoreWeights()),
      score_weights_values_(7, 0.0), ideal_grasp_pose_() {

  XmlRpc::XmlRpcValue xmlval;

  std::size_t i = 0;
  for (const std::string &par :
       {"tranlation_x_score", "tranlation_y_score", "tranlation_z_score",
        "rotation_x_score", "rotation_y_score", "rotation_z_score",
        "overhang_score"}) {
    if (nh_.hasParam(par)) { // existence
      nh_.getParam("object_files", xmlval);
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

  rosparam_shortcuts::get("object_description", nh_, "ideal_grasp_pose",
                          ideal_grasp_pose_);
}
