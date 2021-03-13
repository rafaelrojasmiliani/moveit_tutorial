#include "tools.h"
#include <tf2_eigen/tf2_eigen.h>

namespace tools {

std_msgs::ColorRGBA get_color_msg(double _red, double _green, double _blue,
                                  double _alpha) {
  std_msgs::ColorRGBA result;
  result.r = _red;
  result.g = _green;
  result.b = _blue;
  result.a = _alpha;
  return result;
}

geometry_msgs::Vector3 get_vector3_msg(double _x, double _y, double _z) {
  geometry_msgs::Vector3 result;
  result.x = _x;
  result.y = _y;
  result.z = _z;
  return result;
}

visualization_msgs::Marker get_box_msg(double _scale,
                                       std_msgs::ColorRGBA _color) {
  visualization_msgs::Marker result;

  result.type = visualization_msgs::Marker::CUBE;
  result.scale = get_vector3_msg(_scale, _scale, _scale);
  result.color = _color;
  return result;
}

visualization_msgs::Marker get_gray_box_msg(double _scale) {
  visualization_msgs::Marker result;

  result.type = visualization_msgs::Marker::CUBE;
  result.scale = get_vector3_msg(_scale, _scale, _scale);
  result.color = get_color_msg(0.5, 0.5, 0.5, 1.0);
  return result;
}

visualization_msgs::InteractiveMarker get_interactive_marker_msg(
    const std::string &_name, const std::string &_frame_id,
    const std::string &_description, const Eigen::Vector3d &_position,
    const Eigen::Quaterniond &_orientation, double _scale) {

  visualization_msgs::InteractiveMarker result;
  result.header.frame_id = _frame_id;
  result.header.stamp = ros::Time::now();
  result.pose.position = tf2::toMsg(_position);
  result.pose.orientation = tf2::toMsg(_orientation);
  result.scale = _scale;
  result.name = _name;
  result.description = _description;
  return result;
}

visualization_msgs::InteractiveMarkerControl get_interactive_marker_control_msg(
    const std::string &_name, const std::string &_description,
    const int _orientation_mode, const int _interaction_mode,
    const Eigen::Quaterniond &_orientation, bool _is_visible) {

  visualization_msgs::InteractiveMarkerControl result;

  result.name = _name;
  result.description = _description;
  result.orientation_mode = _orientation_mode;

  result.orientation = tf2::toMsg(_orientation);
  result.interaction_mode = _interaction_mode;
  result.always_visible = _is_visible;
  return result;
}

} // namespace tools
