/* Author: Rafael A. Rojas*/
/*ros::init
  ros::AsyncSpinner 
 * */
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a_robot_model_example_01");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  for(const std::string& param :{"robot_description", "robot_description_semantic"})
    if(!nh.hasParam(param)){
      ROS_ERROR("ROS paramter \"%s\" is not set", param.c_str());
      ros::shutdown();
      return 0;
    }
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  ROS_INFO("robot_model_loader::RobotModelLoader instantiated");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("robot_model_loader::RobotModelLoader instantiated");
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  /*Get the names of all groups that are defined for this mode*/
  const std::vector<std::string>& group_names = kinematic_model->getJointModelGroupNames();
  ROS_INFO("-- Groups in this model ");
  for(const std::string& gname :group_names){
    ROS_INFO("    %s", gname.c_str());
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(gname);
    const std::vector<std::string>& variable_names = joint_model_group->getVariableNames();
    ROS_INFO("    -- variable names in the group %s", gname.c_str());
    for(const std::string& vname :variable_names)
      ROS_INFO("        %s", vname.c_str());
  }

  /* Get the names of the variables that make up the joints that form this
   * state. Fixed joints have no DOF, so
     they are not here, but the variables for mimic joints are included. The
     number of returned elements is always equal to*/
  ros::shutdown();
  return 0;
}
