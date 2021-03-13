
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
} // namespace tools
#endif /* ifndef TOOLS */
