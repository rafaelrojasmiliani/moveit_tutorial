
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
class Orchestrator {
private:
  Orchestrator(const Orchestrator &that);
  Orchestrator &operator=(const Orchestrator &);

public:
  Orchestrator() {

    motion_planning_client_ =
        nh_.serviceClient<std_srvs::Trigger>("plan_random_motion");
    random_objects_client_ =
        nh_.serviceClient<std_srvs::Trigger>("random_object");
  }
  virtual ~Orchestrator() {}

  void spin() {

    motion_planning_client_.waitForExistence();
    random_objects_client_.waitForExistence();

    std_srvs::Trigger trigger_msg;
    while (ros::ok()) {
      ROS_INFO("Adding random object");
      random_objects_client_.call(trigger_msg);
      ros::WallDuration(0.5).sleep();
      ROS_INFO("Calling motion planner");
      motion_planning_client_.call(trigger_msg);
      ros::WallDuration(0.5).sleep();
      ros::spinOnce();
    }
  }

  ros::ServiceClient random_objects_client_;
  ros::ServiceClient motion_planning_client_;
  ros::NodeHandle nh_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "orchestrator");

  Orchestrator orch;

  orch.spin();

  return 0;
}
