#ifndef MYMARKER_H
#define MYMARKER_H

class MyMarker {
private:
  MyMarker(const MyMarker &that);
  MyMarker &operator=(const MyMarker &);
  visualization_msgs::InteractiveMarker imarker_msg_;
  interactive_markers::InteractiveMarkerServer *const server_;

public:
  enum Dof { BOTH, POS, ORIENT };
  enum Axis { X, Y, Z };
  MyMarker(interactive_markers::InteractiveMarkerServer &server,
           const std::string &imarker_name, const std::string &link_name,
           const std::string &frame_id,
           boost::function<void(
               const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)>
               callback,
           Dof dof = BOTH);
  virtual ~MyMarker();
};

#endif /* MYMARKER_H */
