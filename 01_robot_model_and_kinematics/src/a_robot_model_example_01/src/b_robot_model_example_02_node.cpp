/* Author: Rafael A. Rojas*/
/*ros::init
  ros::AsyncSpinner 
 * */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <mutex>

class cRobotModel{
    public:
    cRobotModel(): 
        nh_(), 
        robot_model_loader_("robot_description"), 
        robot_model_(robot_model_loader_.getModel()), 
        kinematic_model_(new robot_state::RobotState(robot_model_)),
        group_name_("myrobotplanninggroup"),
        joint_group_(kinematic_model_->getJointModelGroup(group_name_))
    {
        joint_state_subs_ = nh_.subscribe<sensor_msgs::JointState> ("joint_states", 5, &cRobotModel::setJointStates, this);
    }
    void printDK(){
        joint_mutex_.lock();
        const std::vector<std::string>& variable_names = joint_group_->getVariableNames();
        const std::vector<std::string>& link_names = joint_group_->getLinkModelNames();
        std::vector<double> joint_values;
        kinematic_model_->copyJointGroupPositions(joint_group_, joint_values);
        joint_mutex_.unlock();
        ROS_INFO("-- joints in group %s with their position", group_name_.c_str());
        for (std::size_t i = 0; i < variable_names.size(); ++i)
          ROS_INFO("Joint %s: %f", variable_names[i].c_str(), joint_values[i]);
        for(const std::string& lname :link_names){
          // getGlobalLinkTransform calls updateLinkTransforms
          const Eigen::Isometry3d& link_pose = kinematic_model_->getGlobalLinkTransform(lname);
          ROS_INFO_STREAM(lname << " Translation: \n" << link_pose.translation() << "\n");
          ROS_INFO_STREAM(lname << " Rotation: \n" << link_pose.rotation() << "\n");
        }
        joint_mutex_.unlock();
    }
    private:
    void setJointStates(const sensor_msgs::JointState::ConstPtr &_js){
        joint_mutex_.lock();
        kinematic_model_->setJointGroupPositions(joint_group_, _js->position);
        joint_mutex_.unlock();
    }
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_subs_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    const robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr kinematic_model_;
    std::string group_name_;
    const moveit::core::JointModelGroup* joint_group_;
    std::mutex joint_mutex_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "b_robot_model_example_02");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  
  for(const std::string& param :{"robot_description", "robot_description_semantic"})
    if(!nh.hasParam(param)){
      ROS_ERROR("ROS paramter \"%s\" is not set", param.c_str());
      ros::shutdown();
      return 0;
    }
  {
    cRobotModel my_robot;
    ros::WallDuration(1).sleep();
    my_robot.printDK();
    ros::WallDuration(1).sleep();
    my_robot.printDK();
  }
  ros::WallDuration(0.5).sleep();
  ros::shutdown();
  return 0;
}

