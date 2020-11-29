// moveit_msgs::ApplyPlanningScene, PlanningScene
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h> //moveit_msgs::CollisionObject
#include <shape_msgs/SolidPrimitive.h>   // shape_msgs::SolidPrimitive

#include <ros/ros.h>

double rand_double(const double range);
shape_msgs::SolidPrimitive get_solid_primitive(const std::string &_type);

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

/* Returns a randon object operation */
moveit_msgs::CollisionObject
random_object_msg(std::vector<moveit_msgs::CollisionObject> &object_list) {
  shape_msgs::SolidPrimitive shape;    // object shape
  moveit_msgs::CollisionObject object; // result

  const int add_or_remove = object_list.empty() ? 0 : std::rand() % 2;

  if (add_or_remove == 0) {
    const int shape_type = std::rand() % 4;
    switch (shape_type) {
    case 0:
      shape = get_solid_primitive("box");
      break;
    case 1:
      shape = get_solid_primitive("sphere");
      break;
    case 2:
      shape = get_solid_primitive("cylinder");
      break;
    case 3:
      shape = get_solid_primitive("cone");
      break;
    }
    double x = rand_double(1.0);
    double y = rand_double(1.0);
    double z = rand_double(1.0);
    object = get_colision_object("table", shape, x, y, z);
    object.operation = object.APPEND;
  } else {
    size_t idx = std::rand() % object_list.size();
    object.id = object_list[idx].id;
    object.operation = object.REMOVE;
  }
  return object;
}

double rand_double(const double range) {
  double rand_number =
      (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX));
  return 2.0 * rand_number * range - range;
}

shape_msgs::SolidPrimitive get_solid_primitive(const std::string &_type) {

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
  } else {
    throw;
  }

  return result;
}
