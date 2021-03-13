
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

void append_so3_control_msg_to_interactive_marker_msg(
    visualization_msgs::InteractiveMarker &_imaker) {
  // 1. Instantiate an array with the desired orientations to use
  std::vector<Eigen::Quaterniond> axis_orientation;
  axis_orientation.push_back(Eigen::Quaterniond(1, 1, 0, 0));
  axis_orientation.push_back(Eigen::Quaterniond(1, 0, 0, 1));
  axis_orientation.push_back(Eigen::Quaterniond(1, 0, 1, 0));
  // 2. Instantiate an array with the desired strings
  std::vector<std::string> axis_name;
  axis_name.push_back("move_x");
  axis_name.push_back("move_y");
  axis_name.push_back("move_z");
  // 3. Instantiate an interactive marker control for each
  // direaction/orientation
  for (int i = 0; i < 3; i++) {
    // 3.0. Declare interacive marker control message
    visualization_msgs::InteractiveMarkerControl single_axis_control;
    // 3.1 set the members of interactive marker control as a rotate axis
    // control.
    single_axis_control = get_interactive_marker_control_msg(
        axis_name[i], axis_name[i],
        visualization_msgs::InteractiveMarkerControl::INHERIT,
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS,
        axis_orientation[i], true);
    // 3.3. Associate the rotation interactive marker control to the interactive
    // marker
    _imaker.controls.push_back(single_axis_control);
    // 3.4 set the members of interactive marker control as a axis traslation
    // control.
    single_axis_control = get_interactive_marker_control_msg(
        axis_name[i], axis_name[i],
        visualization_msgs::InteractiveMarkerControl::INHERIT,
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS,
        axis_orientation[i], true);
    // 3.5. Associate the translation interactive marker control to the
    // interactive marker
    _imaker.controls.push_back(single_axis_control);
  }
}
} // namespace tools
