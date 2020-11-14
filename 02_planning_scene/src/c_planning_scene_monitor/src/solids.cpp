
#include <solids.h>
#include <string>

shape_msgs::SolidPrimitive get_primitive(const std::string &_type) {

  shape_msgs::SolidPrimitive result;

  if (_type == "box") {
    result.type = result.BOX;
    result.dimensions.resize(3);
    result.dimensions[0] = 0.1;
    result.dimensions[1] = 0.1;
    result.dimensions[2] = 0.1;
  } else if (_type == "sphere") {
    result.type = result.SPHERE;
    result.dimensions.resize(1);
    result.dimensions[0] = 0.1;
  } else if (_type == "cylinder") {
    result.type = result.CYLINDER;
    result.dimensions.resize(2);
    result.dimensions[0] = 0.1;
    result.dimensions[1] = 0.1;
  } else if (_type == "cone") {
    result.type = result.CONE;
    result.dimensions.resize(2);
    result.dimensions[0] = 0.1;
    result.dimensions[1] = 0.1;
  }

  return result;
}

moveit_msgs::CollisionObject
get_colision_object(const std::string &_header_frame,
                    const shape_msgs::SolidPrimitive &_shape, double _x,
                    double _y, double _z) {
  static unsigned int counter = 0;
  std::string object_name = "object" + std::to_string(counter);
  counter++;

  moveit_msgs::CollisionObject result;

  result.header.frame_id = _header_frame;
  result.id = object_name;

  result.primitives.push_back(_shape);

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = _x;
  pose.position.y = _y;
  pose.position.z = _z;
  result.primitive_poses.push_back(pose);

  return result;
}
