#ifndef OBJECTDESCRIPTION_H
#define OBJECTDESCRIPTION_H
#include <kdl/tree.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_grasps/suction_grasp_data.h>
#include <moveit_grasps/suction_grasp_generator.h>
#include <moveit_grasps/suction_grasp_scorer.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
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
  moveit_grasps::SuctionGraspDataPtr suction_data_;
  moveit_grasps::SuctionGraspScoreWeightsPtr score_weights_;
  std::vector<double> score_weights_values_;

  Eigen::Isometry3d ideal_grasp_pose_;

  moveit::planning_interface::PlanningSceneInterface psi_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  moveit_grasps::SuctionGraspGeneratorPtr grasp_generator_;
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates_;

  urdf::Model urdf_model_;
  KDL::Tree kdl_tree_;
  std::string object_description_file_;
  std::vector<shape_msgs::SolidPrimitive> vector_of_solid_primitives_;
  std::vector<std::string> vector_of_solid_primitive_names_;
  std::string root_link_name_;
  std::vector<Eigen::Isometry3d> vector_of_solid_primitive_origins_;

  ObjectDescription(const ObjectDescription &that);
  ObjectDescription &operator=(const ObjectDescription &);

  std::map<std::string, robot_state_publisher::SegmentPair> segments_fixed_;
  void addChildren(const KDL::SegmentMap::const_iterator segment);

  bool get_shapes_from_urdf();
  void get_transforms();

public:
  ObjectDescription(const std::string &_name);
  virtual ~ObjectDescription();
  Eigen::Isometry3d get_pose();

  const std::string &get_name() const { return name_; };

  void load_urdf();
};

#endif /* OBJECTDESCRIPTION_H */
