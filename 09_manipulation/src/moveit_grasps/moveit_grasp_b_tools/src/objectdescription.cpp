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

#include <algorithm> // remove_if
#include <moveit/collision_detection_bullet/bullet_integration/ros_bullet_utils.h> // urdfPose2Eigen
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

std::string stripSlash(const std::string &in) {
  if (in.size() && in[0] == '/') {
    return in.substr(1);
  }
  return in;
}

shape_msgs::SolidPrimitive get_primitive(const std::string &_type,
                                         std::vector<double> _dimension);
ObjectDescription::ObjectDescription(const std::string &_name)
    : // begin initialization list
      collision_object_(), ee_name_(""), name_(_name), nh_(), nh_object_(_name),
      robot_model_loader_("robot_description"),
      robot_model_(robot_model_loader_.getModel()), suction_data_(nullptr),
      score_weights_(new moveit_grasps::SuctionGraspScoreWeights()),
      score_weights_values_(7, 0.0), ideal_grasp_pose_(), psi_(),
      visual_tools_(new moveit_visual_tools::MoveItVisualTools(
          robot_model_->getModelFrame(), "/rviz_visual_tools")),
      grasp_generator_(nullptr), grasp_candidates_(), urdf_model_(),
      kdl_tree_(), object_description_file_(), vector_of_solid_primitives_(),
      root_link_name_()
// end initialization list
{
  // 0. Declare error handling var
  std::size_t error = 0;
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
    get_shapes_from_urdf();
    ROS_INFO("---------------------------------------\n");
    get_transforms();
    Eigen::Isometry3d initial_pose;
    error += !rosparam_shortcuts::get("object_description", nh_object_,
                                      "initial_pose", initial_pose);

    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(initial_pose, pose);
    collision_object_.primitive_poses.push_back(pose);
    ROS_INFO("number of primitive poses %zu\n",
             collision_object_.primitive_poses.size());
    ROS_INFO("number of primitive shapes %zu\n",
             collision_object_.primitives.size());
    collision_object_.header.frame_id = "world";
    collision_object_.id = _name;
    psi_.applyCollisionObject(collision_object_);
  }
  XmlRpc::XmlRpcValue xmlval;

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

  // -------------------------------
  // --- 3. Push object into scene
  // -------------------------------

  collision_object_.id = name_;
  std::map<std::string, moveit_msgs::CollisionObject> object_pair_list =
      psi_.getObjects({_name});
  if (object_pair_list.find(_name) == object_pair_list.end()) {
    ROS_ERROR("Object not found in plannins scene");
    error += 1;
  } else {
    collision_object_ = object_pair_list[_name];
  }
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

void ObjectDescription::load_urdf() {

  urdf_model_.initFile(object_description_file_);
  kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_);
  std::vector<urdf::LinkSharedPtr> links_in_object;

  urdf_model_.getLinks(links_in_object);
}

void ObjectDescription::addChildren(
    const KDL::SegmentMap::const_iterator segment) {

  const std::string &root = GetTreeElementSegment(segment->second).getName();

  // get children of the segment
  const std::vector<KDL::SegmentMap::const_iterator> &children =
      GetTreeElementChildren(segment->second);

  for (size_t i = 0; i < children.size(); ++i) {
    const KDL::Segment &child = GetTreeElementSegment(children[i]->second);
    robot_state_publisher::SegmentPair s(
        GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
      addChildren(children[i]);
    }
  }
}
void ObjectDescription::get_transforms() {
  tf2_ros::Buffer tf_buffer;
  // 1. populate segments_fixed_
  addChildren(kdl_tree_.getRootSegment());
  // 2. for each segment compute the required transform
  for (std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator
           segment_pair = segments_fixed_.begin();
       segment_pair != segments_fixed_.end(); segment_pair++) {
    // Get the reference of the link represented by the kdl_segment
    const urdf::LinkConstSharedPtr &link =
        urdf_model_.getLink(segment_pair->second.segment.getName());

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
  // 3. for all links in the collision object list get push it position
  collision_object_.primitives = vector_of_solid_primitives_;
  for (int i = 1; i < vector_of_solid_primitives_.size(); i++) {

    if (tf_buffer.canTransform(root_link_name_,
                               vector_of_solid_primitive_names_[i],
                               ros::Time::now())) {

      ROS_INFO("---------\n");
      geometry_msgs::TransformStamped primitive_transform =
          tf_buffer.lookupTransform(vector_of_solid_primitive_names_[i],
                                    root_link_name_, ros::Time::now());
      ROS_INFO("---------\n");
      tf2::doTransform(vector_of_solid_primitive_origins_[i],
                       vector_of_solid_primitive_origins_[i],
                       primitive_transform);
      ROS_INFO("---------\n");
      geometry_msgs::Pose pose;
      pose = tf2::toMsg(vector_of_solid_primitive_origins_[i]);
      ROS_INFO("---------\n");
      collision_object_.primitive_poses.push_back(pose);
      ROS_INFO("---------\n");
    } else {
      ROS_INFO("cannot find a tranform from %s to %s\n",
               root_link_name_.c_str(),
               vector_of_solid_primitive_names_[i].c_str());
    }
  }
}

bool ObjectDescription::get_shapes_from_urdf() {

  urdf_model_.initFile(object_description_file_);
  kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_);
  std::vector<urdf::LinkSharedPtr> links_in_object;

  const std::string &root_name =
      GetTreeElementSegment(kdl_tree_.getRootSegment()->second).getName();
  ROS_INFO("--------------\n");
  ROS_INFO("-------------- %s \n", root_name.c_str());
  ROS_INFO("--------------\n");

  urdf_model_.getLinks(links_in_object);
  std::vector<urdf::LinkSharedPtr>::iterator it =
      find_if(links_in_object.begin(), links_in_object.end(),
              [&root_name](const urdf::LinkSharedPtr &_link) {
                return _link->name == root_name;
              });

  urdf::LinkSharedPtr root_link = *it;

  root_link_name_ = root_link->name;

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
    vector_of_solid_primitives_.push_back(std::move(solid_primitive));
    vector_of_solid_primitive_names_.push_back(link->name);
    ROS_INFO("+++ name %s ++++\n", link->name.c_str());
    vector_of_solid_primitive_origins_.push_back(
        collision_detection_bullet::urdfPose2Eigen(pose));
  }
  return true;
}
