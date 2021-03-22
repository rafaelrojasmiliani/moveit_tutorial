/* Author: Rafael A. Rojas*/
/*ros::init
  ros::AsyncSpinner
 * */
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_monitor/current_state_monitor.h> // CurrentStateMonitor
#include <moveit/robot_model/robot_model.h> //robot_model::RobotModel
#include <moveit/robot_model_loader/robot_model_loader.h> // robot_model_loader::RobotModelLoader
#include <moveit/robot_state/robot_state.h> // robot_state::RobotStatePtr
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h> // tf2_ros::Buffer

int main(int argc, char **argv) {
  ros::ServiceClient random_objects_client_;
  ros::ServiceClient motion_planning_client_;
  std_srvs::Trigger trigger_msg;
  ros::NodeHandle nh;
  // 0. Initialiye node
  ros::init(argc, argv, "node");
  // 1. Create TF buffer
  const std::shared_ptr<tf2_ros::Buffer> &tf_buffer =
      std::shared_ptr<tf2_ros::Buffer>();
  // 2. Instantiate robot loader
  robot_model_loader::RobotModelLoader my_robot_model_loader(
      "robot_description");
  // 3. Instantiate CurrentStateMonitor
  planning_scene_monitor::CurrentStateMonitor csm(
      my_robot_model_loader.getModel(), tf_buffer);
  motion_planning_client_ =
      nh.serviceClient<std_srvs::Trigger>("plan_random_motion");
  random_objects_client_ = nh.serviceClient<std_srvs::Trigger>("random_object");
  while (ros::ok()) {
    random_objects_client_.call(trigger_msg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
  }
  return 0;
}
