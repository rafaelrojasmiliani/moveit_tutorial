#ifndef OBJECTDESCRIPTION_H
#define OBJECTDESCRIPTION_H

class ObjectDescription {
private:
  std::shared_ptr<moveit_grasps::SuctionGraspScoreWeights> score_weights_;
  moveit_grasps::SuctionGraspDataPtr suction_data_;
  moveit_msgs::CollisionObject collision_object_;

public:
  ObjectDescription();
  ObjectDescription(const ObjectDescription &that);
  ObjectDescription &operator=(const ObjectDescription &);
  virtual ~ObjectDescription();
  Eigen::Isometry3d get_pose() const;

  const std::string &get_name() const;
};

#endif /* OBJECTDESCRIPTION_H */
