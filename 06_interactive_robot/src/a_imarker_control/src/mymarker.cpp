#include <mymarker.h>
MyMarker::MyMarker(
    interactive_markers::InteractiveMarkerServer &server,
    const std::string &imarker_name, const std::string &link_name,
    const std::string &frame_id,
    boost::function<
        void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)>
        callback,
    Dof dof = BOTH) {

  imarker_msg_.header.frame_id = frame_id;
  imarker_msg_.pose.position = tf2::toMsg(position);
  imarker_msg_.pose.orientation = tf2::toMsg(orientation);
  imarker_msg_.scale = 0.3;

  imarker_msg_.name = name;
  imarker_msg_.description = name;

  // insert a control with a marker
  switch (dof) {
  case POS:
    makeBallControl();
    break;
  case ORIENT:
  case BOTH:
  default:
    makeAxisControl();
    break;
  }

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation_mode =
      visualization_msgs::InteractiveMarkerControl::FIXED;

  // add orientation and/or position controls
  for (int i = 0; i < 3; i++) {
    static const char *dirname = "xyz";

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 0;
    switch (i) {
    case 0:
      control.orientation.x = 1;
      break;
    case 1:
      control.orientation.y = 1;
      break;
    case 2:
      control.orientation.z = 1;
      break;
    }

    if (dof == ORIENT || dof == BOTH) {
      control.name = std::string("rotate_") + std::string(1, dirname[i]);
      control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      imarker_msg_.controls.push_back(control);
    }
    if (dof == POS || dof == BOTH) {
      control.name = std::string("move_") + std::string(1, dirname[i]);
      control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      imarker_msg_.controls.push_back(control);
    }
  }

  // tell the server to show the marker and listen for changes
  server_->insert(imarker_msg_);
  server_->setCallback(imarker_msg_.name, std::move(callback));
  server_->applyChanges();
}
virtual ~MyMarker();
