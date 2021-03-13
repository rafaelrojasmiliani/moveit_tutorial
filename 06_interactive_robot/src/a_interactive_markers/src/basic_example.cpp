#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO_STREAM(feedback->marker_name << " is now at "
                                        << feedback->pose.position.x << ", "
                                        << feedback->pose.position.y << ", "
                                        << feedback->pose.position.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_marker");

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "Simple 1-DOF Control";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  // add the control to the interactive marker
  int_marker.controls.push_back(box_control);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // add the control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}
visualization_msgs::Marker get_box_marker_msg() {
  // create a grey box marker
  visualization_msgs::Marker result;
  result.type = visualization_msgs::Marker::CUBE;
  result.scale.x = 0.45;
  result.scale.y = 0.45;
  result.scale.z = 0.45;
  result.color.r = 0.5;
  result.color.g = 0.5;
  result.color.b = 0.5;
  result.color.a = 1.0;
  return result;
}
/*
visualization_msgs::InteractiveMarker get_interactive_marker_msg(
    const std::string &_name, const std::string &_frame_id,
    const Eigen::Vector3d &_position, const Eigen::Quaterniond &_orientation,
    double _scale) {

  visualization_msgs::InteractiveMarker result;
  result.header.frame_id = _frame_id;
  result.pose.position = tf2::toMsg(_position);
  result.pose.orientation = tf2::toMsg(_orientation);
  result.scale = _scale;
  result.name = _name;
  result.description = _name;
}*/
