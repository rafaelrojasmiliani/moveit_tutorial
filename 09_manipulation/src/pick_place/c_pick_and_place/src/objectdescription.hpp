
#ifndef OBJECTDESCRIPTION_H
#define OBJECTDESCRIPTION_H
#include <kdl/tree.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GripperTranslation.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <trajectory_msgs/JointTrajectory.h>
#include <urdf/model.h>
#include <vector>
// moveit::planning_interface::PlanningSceneInterface

class ObjectDescription {
private:
  moveit_msgs::CollisionObject collision_object_;
  std::string ee_name_;
  const std::string name_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_object_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  const robot_model::RobotModelPtr &robot_model_;
  std::vector<double> score_weights_values_;

  std::vector<moveit_msgs::Grasp> grasp_candidates_;

  moveit_msgs::GripperTranslation approach_gripper_translation_;
  moveit_msgs::GripperTranslation retreat_gripper_translation_;

  trajectory_msgs::JointTrajectory gripper_closure_;
  trajectory_msgs::JointTrajectory gripper_opening_;

  moveit::planning_interface::PlanningSceneInterface psi_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  urdf::Model urdf_model_;
  std::string object_description_file_;

  ObjectDescription(const ObjectDescription &that);
  ObjectDescription &operator=(const ObjectDescription &);

  std::map<std::string, robot_state_publisher::SegmentPair> segments_fixed_;

  void read_sub_frames();

  bool read_gripper_translation(moveit_msgs::GripperTranslation &_gt,
                                const std::string _param);

  bool read_gripper_pose(trajectory_msgs::JointTrajectory &_jt,
                         const std::string _param);

public:
  ObjectDescription(const std::string &_name);
  virtual ~ObjectDescription();
  Eigen::Isometry3d get_pose();

  const std::string &get_name() const { return name_; };

  std::vector<moveit_msgs::Grasp> &compute_grasp_candidates();
  moveit_msgs::Grasp ideal_grasp_;
  Eigen::Isometry3d ideal_grasp_pose_;
  Eigen::Isometry3d tcp_to_eef_mount_transform_;
};

#endif /* OBJECTDESCRIPTION_H */
