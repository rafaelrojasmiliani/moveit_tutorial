#ifndef TOOLS
#define TOOLS value
#include <Eigen/Geometry>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
namespace tools {
std_msgs::ColorRGBA get_color_msg(double _red, double _green, double _blue,
                                  double _alpha);

geometry_msgs::Vector3 get_vector3_msg(double _x, double _y, double _z);

visualization_msgs::Marker get_box_msg(double _scale,
                                       std_msgs::ColorRGBA _color);

visualization_msgs::Marker get_gray_box_msg(double _scale);

visualization_msgs::InteractiveMarker get_interactive_marker_msg(
    const std::string &_name, const std::string &_frame_id,
    const std::string &_description, const Eigen::Vector3d &_position,
    const Eigen::Quaterniond &_orientation, double _scale);

visualization_msgs::InteractiveMarkerControl get_interactive_marker_control_msg(
    const std::string &_name, const std::string &_description,
    const int _orientation_mode, const int _interaction_mode,
    const Eigen::Quaterniond &_orientation, bool _is_visible);

} // namespace tools
#endif /* ifndef TOOLS */
