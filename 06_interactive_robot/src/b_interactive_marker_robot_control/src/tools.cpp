
#include "tools.h"
#include <tf2_eigen/tf2_eigen.h>
namespace tools {

visualization_msgs::Marker get_axis_marker(Axis axis) {
  visualization_msgs::Marker marker;
  double scale = 0.1;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = scale * 0.15;
  marker.scale.y = scale * 0.15;
  marker.scale.z = scale;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  switch (axis) {
  case X:
    marker.pose.position.x = 0.5 * scale;
    marker.pose.orientation.y = 1.0;
    marker.color.r = 0.5;
    break;
  case Y:
    marker.pose.position.y = 0.5 * scale;
    marker.pose.orientation.x = 1.0;
    marker.color.g = 0.5;
    break;
  case Z:
  default:
    marker.pose.position.z = 0.5 * scale;
    marker.color.b = 0.5;
    break;
  }
  return marker;
}

visualization_msgs::InteractiveMarkerControl get_control() {
  visualization_msgs::InteractiveMarkerControl control;

  control.always_visible = true;
  /*control.markers.push_back(get_axis_marker(X));
  control.markers.push_back(get_axis_marker(Y));
  control.markers.push_back(get_axis_marker(Z));*/
  control.name = std::string("rotate_");
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  return control;
}

visualization_msgs::InteractiveMarker
get_imarker_msg(const std::string &_name, const std::string &_frame_id,
                const Eigen::Vector3d &_position,
                const Eigen::Quaterniond &_orientation, double _scale) {

  visualization_msgs::InteractiveMarker result;
  result.header.frame_id = _frame_id;
  result.pose.position = tf2::toMsg(_position);
  result.pose.orientation = tf2::toMsg(_orientation);
  result.scale = _scale;
  result.name = _name;
  result.description = _name;
  return result;
}
} // namespace tools
