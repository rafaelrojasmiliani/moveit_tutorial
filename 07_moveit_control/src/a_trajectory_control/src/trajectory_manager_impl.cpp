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
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h> // tf2_ros::Buffer

int main(int argc, char **argv) {
  // 0. Initialiye node and variables
  ros::init(argc, argv, "node");
  ros::ServiceClient random_objects_client_;
  ros::ServiceClient motion_planning_client_;
  std_srvs::Trigger trigger_msg;
  moveit_msgs::GetMotionPlan get_motion_plan_msg;
  ros::NodeHandle nh;
  // 1. Create TF buffer
  const std::shared_ptr<tf2_ros::Buffer> &tf_buffer =
      std::shared_ptr<tf2_ros::Buffer>();
  // 2. Instantiate robot loader
  robot_model_loader::RobotModelLoader my_robot_model_loader(
      "robot_description");
  // 2.1 Instantiate robot model
  robot_model::RobotModelConstPtr my_model = my_robot_model_loader.getModel();
  // 3. Instantiate CurrentStateMonitor
  planning_scene_monitor::CurrentStateMonitorPtr csm(
      new planning_scene_monitor::CurrentStateMonitor(my_model, tf_buffer));

  // 4. Instantiate Trajectory Execution Manager
  trajectory_execution_manager::TrajectoryExecutionManager tem(my_model, csm);
  random_objects_client_ = nh.serviceClient<std_srvs::Trigger>("random_object");
  motion_planning_client_ =
      nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_random_motion");
  while (ros::ok()) {
    random_objects_client_.call(trigger_msg);
    ros::WallDuration(0.5).sleep();
    motion_planning_client_.call(get_motion_plan_msg);
    if (get_motion_plan_msg.response.motion_plan_response.error_code.val == 1)
      tem.pushAndExecute(
          get_motion_plan_msg.response.motion_plan_response.trajectory);
    ros::spinOnce();
  }
  return 0;
}
