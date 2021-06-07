
#include <algorithm>                     // iter_swap
#include <eigen_conversions/eigen_msg.h> // poseMsgToEigen
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <moveit/collision_detection_bullet/bullet_integration/ros_bullet_utils.h> // urdfPose2Eigen
#include <moveit_msgs/CollisionObject.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_eigen/tf2_eigen.h> // eigenToTransform
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/buffer.h>
#include <urdf_model/link.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <kdl/tree.hpp>
#include <urdf/model.h>

#include <map>

#include <utility>

namespace urdf_to_collision_object {

void addChildren(
    const KDL::SegmentMap::const_iterator _segment,
    std::map<std::string, robot_state_publisher::SegmentPair> &_segments_fixed);

std::string stripSlash(const std::string &in) {
  if (in.size() && in[0] == '/') {
    return in.substr(1);
  }
  return in;
}

bool urdf_to_collision_object(urdf::Model &_urdf_model,
                              moveit_msgs::CollisionObject &_result) {

  KDL::Tree kdl_tree;
  tf2_ros::Buffer tf_buffer;
  // ----------------------------------
  // -------  Part I ----------
  //  Get the links from the urdf model and extract its geometry.
  //  output: an array with names and sapes
  // ----------------------------------
  //
  std::map<std::string,
           std::pair<shape_msgs::SolidPrimitive, Eigen::Isometry3d>>
      solid_primitives_map;

  kdl_parser::treeFromUrdfModel(_urdf_model, kdl_tree);
  std::vector<urdf::LinkSharedPtr> links_in_object;

  const std::string &root_link_name =
      GetTreeElementSegment(kdl_tree.getRootSegment()->second).getName();

  _urdf_model.getLinks(links_in_object);

  std::vector<urdf::LinkSharedPtr>::iterator it =
      find_if(links_in_object.begin(), links_in_object.end(),
              [&root_link_name](const urdf::LinkSharedPtr &_link) {
                return _link->name == root_link_name;
              });

  urdf::LinkSharedPtr root_link = *it;

  links_in_object.erase(it);

  links_in_object.insert(links_in_object.begin(), root_link);

  shape_msgs::SolidPrimitive solid_primitive;

  for (const urdf::LinkSharedPtr &link : links_in_object) {
    urdf::GeometrySharedPtr geometry;
    urdf::Pose pose;

    if (link->collision) {
      geometry = link->collision->geometry;
      pose = link->collision->origin;
    } else if (link->visual) {
      geometry = link->visual->geometry;
      pose = link->visual->origin;
    } else {
      ROS_INFO("geometry is null\n");
      return false;
    }
    switch (geometry->type) {
    case urdf::Geometry::BOX: {
      // set type
      solid_primitive.type = solid_primitive.BOX;
      // downcast
      const urdf::BoxConstSharedPtr box =
          std::static_pointer_cast<const urdf::Box>(geometry);
      // set dimensions
      solid_primitive.dimensions =
          std::vector<double>{box->dim.x, box->dim.y, box->dim.z};
      break;
    }
    case urdf::Geometry::CYLINDER: {
      // set type
      solid_primitive.type = solid_primitive.CYLINDER;
      // downcast
      const urdf::CylinderConstSharedPtr cylinder =
          std::static_pointer_cast<const urdf::Cylinder>(geometry);
      // set dimensions
      solid_primitive.dimensions =
          std::vector<double>{cylinder->length, cylinder->radius};
      break;
    }
    case urdf::Geometry::SPHERE: {
      // set type
      solid_primitive.type = solid_primitive.SPHERE;
      // downcast
      const urdf::SphereConstSharedPtr sphere =
          std::static_pointer_cast<const urdf::Sphere>(geometry);
      // set dimensions
      solid_primitive.dimensions = std::vector<double>{sphere->radius};
      break;
    }
    case urdf::Geometry::MESH: {
      break;
    }
    }
    solid_primitives_map[link->name] = std::make_pair(
        solid_primitive, collision_detection_bullet::urdfPose2Eigen(pose));
  }
  // ----------------------------------
  // -------  Part II ----------
  // Get the relative positions of the shapes
  // ----------------------------------

  // II.1 build the array of segmenets
  std::map<std::string, robot_state_publisher::SegmentPair> segments;

  addChildren(kdl_tree.getRootSegment(), segments);

  // 2. for each segment compute the required transform
  for (std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator
           segment_pair = segments.begin();
       segment_pair != segments.end(); segment_pair++) {
    // Get the reference of the link represented by the kdl_segment
    const urdf::LinkConstSharedPtr &link =
        _urdf_model.getLink(segment_pair->second.segment.getName());

    geometry_msgs::TransformStamped tf_transform =
        tf2::kdlToTransform(segment_pair->second.segment.pose(0));
    tf_transform.header.stamp = ros::Time::now();

    tf_transform.header.frame_id = stripSlash(segment_pair->second.root);
    tf_transform.child_frame_id = stripSlash(segment_pair->second.tip);
    // 2.1 push the transform into the tf_buffer
    if (not tf_buffer.setTransform(tf_transform, "the_boss", true)) {
      ROS_INFO("----- CANNOT SET TRANSFORM\n");
    }
  }

  std::size_t counter = 0;    // use to fid the position of the root link
  std::size_t root_index = 0; // use to fid the position of the root link
  for (const std::pair<std::string,
                       std::pair<shape_msgs::SolidPrimitive, Eigen::Isometry3d>>
           &it : solid_primitives_map) {

    if (it.first == root_link_name)
      root_index = counter;
    counter++;
    if (tf_buffer.canTransform(root_link_name, it.first, ros::Time::now())) {

      geometry_msgs::TransformStamped primitive_transform =
          tf_buffer.lookupTransform(root_link_name, it.first, ros::Time::now());

      Eigen::Isometry3d aux_isometry;
      tf2::doTransform(it.second.second, aux_isometry, primitive_transform);
      geometry_msgs::Pose pose;
      pose = tf2::toMsg(aux_isometry);
      _result.primitive_poses.push_back(pose);
      _result.primitives.push_back(it.second.first);

    } else {
      ROS_ERROR("cannot find a tranform from %s to %s\n",
                root_link_name.c_str(), it.first.c_str());
    }
  }
  std::iter_swap(_result.primitive_poses.begin(),
                 _result.primitive_poses.begin() + root_index);
  std::iter_swap(_result.primitives.begin(),
                 _result.primitives.begin() + root_index);
  return true;
}

void addChildren(const KDL::SegmentMap::const_iterator _segment,
                 std::map<std::string, robot_state_publisher::SegmentPair>
                     &_segments_fixed) {

  const std::string &root = GetTreeElementSegment(_segment->second).getName();

  // get children of the segment
  const std::vector<KDL::SegmentMap::const_iterator> &children =
      GetTreeElementChildren(_segment->second);

  for (size_t i = 0; i < children.size(); ++i) {
    const KDL::Segment &child = GetTreeElementSegment(children[i]->second);
    robot_state_publisher::SegmentPair s(
        GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      _segments_fixed.insert(make_pair(child.getJoint().getName(), s));
      addChildren(children[i], _segments_fixed);
    }
  }
}

void update_frames(const Eigen::Isometry3d &_base_pose,
                   moveit_msgs::CollisionObject &_result) {

  geometry_msgs::Pose pose;
  geometry_msgs::TransformStamped base_transform =
      tf2::eigenToTransform(_base_pose);

  tf::poseEigenToMsg(_base_pose, pose);
  _result.primitive_poses[0] = pose;
  for (std::size_t i = 1; i < _result.primitive_poses.size(); i++) {

    tf2::doTransform(_result.primitive_poses[i], _result.primitive_poses[i],
                     base_transform);
  }

  for (std::size_t i = 0; i < _result.subframe_poses.size(); i++) {

    tf2::doTransform(_result.subframe_poses[i], _result.subframe_poses[i],
                     base_transform);
  }
}
} // namespace urdf_to_collision_object
