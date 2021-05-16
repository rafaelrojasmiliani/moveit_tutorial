
#include <moveit/move_group_interface/move_group_interface.h> //moveit::planning_interface::MoveGroupInterface
#include <moveit/robot_model/robot_model.h> //robot_model::RobotModel
#include <moveit/robot_model_loader/robot_model_loader.h> // robot_model_loader::RobotModelLoader
#include <moveit/robot_state/robot_state.h> // robot_state::RobotStatePtr

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader my_robot_model_loader(
      "robot_description");
  const robot_model::RobotModelPtr &my_robot_model =
      my_robot_model_loader.getModel();
  const std::string planning_group_name =
      my_robot_model->getJointModelGroupNames().back();

  moveit::planning_interface::MoveGroupInterface mgi(planning_group_name);

  robot_state::RobotStatePtr rs = mgi.getCurrentState();

  // Raw pointers are frequently used to refer to the planning group for
  // improved performance.
  const robot_state::JointModelGroup *joint_model_group =
      rs->getJointModelGroup(planning_group_name);

  ROS_INFO_NAMED("tutorial", "Planning frame: %s",
                 mgi.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s",
                 mgi.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(mgi.getJointModelGroupNames().begin(),
            mgi.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  ROS_INFO_NAMED("tutorial", "DOF of the joint group %u",
                 mgi.getVariableCount());

  ROS_INFO_NAMED("tutorial", "Planner Id = %s", mgi.getPlannerId().c_str());
  ros::shutdown();
  return 0;
}
