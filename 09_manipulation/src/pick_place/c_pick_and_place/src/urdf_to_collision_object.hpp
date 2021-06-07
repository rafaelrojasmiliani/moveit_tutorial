
#ifndef URDF_TO_COLLSION_OBJECT
#define URDF_TO_COLLSION_OBJECT
#include <Eigen/Geometry>
#include <map>
#include <moveit_msgs/CollisionObject.h>
#include <urdf/model.h>

namespace urdf_to_collision_object {

bool urdf_to_collision_object(urdf::Model &_urdf_model,
                              moveit_msgs::CollisionObject &_result);
void update_frames(const Eigen::Isometry3d &_base_pose,
                   moveit_msgs::CollisionObject &_result);
} // namespace urdf_to_collision_object
#endif /* ifndef URDF_TO_COLLSION_OBJECT */
