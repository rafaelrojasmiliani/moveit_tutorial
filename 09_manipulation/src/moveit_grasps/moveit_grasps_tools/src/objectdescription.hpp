#ifndef OBJECTDESCRIPTION_H
#define OBJECTDESCRIPTION_H
#include <moveit_grasps/suction_grasp_data.h>
#include <moveit_grasps/suction_grasp_scorer.h>

class ObjectDescription {
private:
  moveit_msgs::CollisionObject collision_object_;
  const std::string ee_name_;
  ros::NodeHandle nh_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  const robot_model::RobotModelPtr &robot_model_;
  moveit_grasps::SuctionGraspDataPtr suction_data_;
  moveit_grasps::SuctionGraspScoreWeightsPtr score_weights_;
  std::vector<double> score_weights_values_;

  Eigen::Isometry3d ideal_grasp_pose_;
  ObjectDescription(const ObjectDescription &that);
  ObjectDescription &operator=(const ObjectDescription &);

public:
  ObjectDescription(moveit_msgs::CollisionObject &_collision_object,
                    const std::string &_ee_name, const std::string &_ns);
  virtual ~ObjectDescription();
  Eigen::Isometry3d get_pose() const;

  const std::string &get_name() const;

  void set_ideal_grasp_pose(const Eigen::Isometry3d &_pose);
  void set_grasp_suction_score(double _x, double _y, double _z, double _raw,
                               double _pich, double _yaw, double _overhang);
};

#endif /* OBJECTDESCRIPTION_H */
