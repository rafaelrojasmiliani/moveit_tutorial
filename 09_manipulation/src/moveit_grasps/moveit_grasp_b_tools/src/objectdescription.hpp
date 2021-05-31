#ifndef OBJECTDESCRIPTION_H
#define OBJECTDESCRIPTION_H
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_grasps/suction_grasp_data.h>
#include <moveit_grasps/suction_grasp_generator.h>
#include <moveit_grasps/suction_grasp_scorer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
// moveit::planning_interface::PlanningSceneInterface

class ObjectDescription {
private:
  moveit_msgs::CollisionObject collision_object_;
  std::string ee_name_;
  const std::string name_;
  ros::NodeHandle nh_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  const robot_model::RobotModelPtr &robot_model_;
  moveit_grasps::SuctionGraspDataPtr suction_data_;
  moveit_grasps::SuctionGraspScoreWeightsPtr score_weights_;
  std::vector<double> score_weights_values_;

  Eigen::Isometry3d ideal_grasp_pose_;

  moveit::planning_interface::PlanningSceneInterface psi_;
  // tf publishing thread control
  bool keep_running_;
  std::thread thread_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  moveit_grasps::SuctionGraspGeneratorPtr grasp_generator_;
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates_;

  ObjectDescription(const ObjectDescription &that);
  ObjectDescription &operator=(const ObjectDescription &);

  void publishing_thread();

public:
  ObjectDescription(const std::string &_name);
  virtual ~ObjectDescription();
  Eigen::Isometry3d get_pose();

  const std::string &get_name() const { return name_; };
};

#endif /* OBJECTDESCRIPTION_H */
