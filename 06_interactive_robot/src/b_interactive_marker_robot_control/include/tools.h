
#ifndef TOOLS
#define TOOLS value
#include <Eigen/Geometry>
#include <interactive_markers/interactive_marker_server.h>
#include <utility>
namespace tools {

enum Dof { BOTH, POS, ORIENT };

enum Axis { X, Y, Z };

visualization_msgs::Marker get_axis_marker(Axis axis);

visualization_msgs::InteractiveMarkerControl get_control();

visualization_msgs::InteractiveMarker
get_imarker_msg(const std::string &_name, const std::string &_frame_id,
                const Eigen::Vector3d &_position,
                const Eigen::Quaterniond &_orientation, double _scale);

visualization_msgs::InteractiveMarkerControl get_interactive_marker_control_msg(
    const std::string &_name, const std::string &_description,
    const int _orientation_mode, const int _interaction_mode,
    const Eigen::Quaterniond &_orientation, bool _is_visible);

void append_so3_control_msg_to_interactive_marker_msg(
    visualization_msgs::InteractiveMarker &_imaker);
} // namespace tools
#endif /* ifndef TOOLS */
