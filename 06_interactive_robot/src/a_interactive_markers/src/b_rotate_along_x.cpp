#include <ros/ros.h>

#include "tools.h"
#include <interactive_markers/interactive_marker_server.h>

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO_STREAM(feedback->marker_name << " is now at "
                                        << feedback->pose.position.x << ", "
                                        << feedback->pose.position.y << ", "
                                        << feedback->pose.position.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "b_rotate_along_x");

  Eigen::Quaterniond orientation(1, 0, 0, 0);
  Eigen::Vector3d position(0, 0, 0);
  // 1. Instantiate interactive marker server
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // 2. Instantiate an interactive marker
  visualization_msgs::InteractiveMarker int_marker;
  int_marker = tools::get_interactive_marker_msg("my_marker", "base_link",
                                                 "desc: Simple 1-dof control",
                                                 position, orientation, 1.0);

  // 3. Instantiate a Box marker to display a box
  visualization_msgs::Marker box_marker = tools::get_gray_box_msg(0.45);

  // 4. Intantiate a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl non_interactive_control;
  non_interactive_control = tools::get_interactive_marker_control_msg(
      "", "", visualization_msgs::InteractiveMarkerControl::INHERIT,
      visualization_msgs::InteractiveMarkerControl::NONE, orientation, true);
  // 5. Associate the box to the non interactive control
  non_interactive_control.markers.push_back(box_marker);

  // 6. Associate the non interactive control to the interactive marker
  int_marker.controls.push_back(non_interactive_control);

  // 7. Instantate a second control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control = tools::get_interactive_marker_control_msg(
      "move_x", "move along x",
      visualization_msgs::InteractiveMarkerControl::INHERIT,
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, orientation,
      true);
  // 8. Associate the new control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  // 9. Att the interactive marker to the interactive marker server and
  //    tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 10. Update the changes and send to all clients
  server.applyChanges();

  // 11. Start the ROS main loop
  ros::spin();
}
