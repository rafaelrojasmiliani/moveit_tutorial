/* Author: Rafael A. Rojas*/
/*ros::init
  ros::AsyncSpinner 
 * */
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h> // robot_model_loader::RobotModelLoader
#include <moveit/robot_model/robot_model.h> //robot_model::RobotModel
#include <moveit/robot_state/robot_state.h>

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
  robot_model_loader::RobotModelLoader my_robot_model_loader("robot_description");
  ROS_INFO("robot_model_loader::RobotModelLoader instantiated");
  const robot_model::RobotModelPtr& my_robot_model = my_robot_model_loader.getModel();
  ROS_INFO("robot_model_loader::RobotModelLoader instantiated");
  ROS_INFO("Model frame: %s", my_robot_model->getModelFrame().c_str());

  /*Get the names of all groups that are defined for this mode*/
  {
    const std::vector<std::string>& group_names = my_robot_model->getJointModelGroupNames();
    ROS_INFO("-- Groups in this model ");
    for(const std::string& gname :group_names){
      bool is_end_effector = my_robot_model->hasEndEffector(gname);
      ROS_INFO("    %s which is %s an end-effector", gname.c_str(), is_end_effector?"":"not");
      const moveit::core::JointModelGroup* joint_model_group = my_robot_model->getJointModelGroup(gname);
      ROS_INFO("    -- variable (join) names in the group %s", gname.c_str());
      const std::vector<std::string>& variable_names = joint_model_group->getVariableNames();
      for(const std::string& vname :variable_names)
        ROS_INFO("        %s", vname.c_str());
      ROS_INFO("    -- link names in the group %s", gname.c_str());
      const std::vector<std::string>& link_names = joint_model_group->getLinkModelNames();
      for(const std::string& lname :link_names)
        ROS_INFO("        %s", lname.c_str());
    }
  }

  robot_state::RobotStatePtr my_kinematic_model(new robot_state::RobotState(my_robot_model));
  {
    const std::vector<std::string>& group_names = my_robot_model->getJointModelGroupNames();
    for(const std::string& gname :group_names){
      const moveit::core::JointModelGroup* joint_model_group = my_kinematic_model->getJointModelGroup(gname);
      const std::vector<std::string>& variable_names = joint_model_group->getVariableNames();
      const std::vector<std::string>& link_names = joint_model_group->getLinkModelNames();
      std::vector<double> joint_values;
      my_kinematic_model->copyJointGroupPositions(joint_model_group, joint_values);
      ROS_INFO("-- joints in group %s with their position", gname.c_str());
      for (std::size_t i = 0; i < variable_names.size(); ++i)
        ROS_INFO("Joint %s: %f", variable_names[i].c_str(), joint_values[i]);
      for(const std::string& lname :link_names){
        // getGlobalLinkTransform calls updateLinkTransforms
        const Eigen::Isometry3d& link_pose = my_kinematic_model->getGlobalLinkTransform(lname);
        ROS_INFO_STREAM(lname << " Translation: \n" << link_pose.translation() << "\n");
        ROS_INFO_STREAM(lname << " Rotation: \n" << link_pose.rotation() << "\n");
      }
    }
  }
  /* Get the names of the variables that make up the joints that form this
   * state. Fixed joints have no DOF, so
     they are not here, but the variables for mimic joints are included. The
     number of returned elements is always equal to*/
  ros::shutdown();
  return 0;
}
