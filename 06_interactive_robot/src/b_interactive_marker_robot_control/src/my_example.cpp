
#include "my_interactive_robot.h"
#include <ros/ros.h>
int main(int argc, char **argv) {
  ros::init(argc, argv, "interactivity_tutorial");
  ros::NodeHandle nh;

  MyInteractiveRobot robot("base_link", "myrobotplanninggroup", "ee_link");

  robot.spin();

  return 0;
}
