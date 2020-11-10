
/* Author: Rafael A. Rojas*/

#include <ros/ros.h> // ros::init, ros::AsyncSpinner, 
#include <sensor_msgs/JointState.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "a_planning_scene_class_example");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::size_t count = 0;
  ros::NodeHandle nh;

  // 1) instantiate the  MoveIt robot model
  ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("joint_states_cmd", 1);
  sensor_msgs::JointState js;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // 2) instantiate the  MoveIt PlanningScene with the moveit model
  robot_model::RobotModelPtr my_robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(my_robot_model);

  // 3) instantiate the input and output of the collision checking request
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_detection::CollisionResult collision_result;

  while(ros::ok()){
    robot_state::RobotState& kinematic_model_actual = planning_scene.getCurrentStateNonConst();
    const robot_model::JointModelGroup* joint_group = kinematic_model_actual.getJointModelGroup("myrobotplanninggroup");
    collision_request.group_name = "myrobotplanninggroup";
    collision_result.clear();
    kinematic_model_actual.setToRandomPositions(joint_group);
    planning_scene.checkSelfCollision(collision_request, collision_result);

    kinematic_model_actual.copyJointGroupPositions(joint_group, js.position);
    js.name = joint_group->getVariableNames();
    js_pub.publish(js);

    if(collision_result.collision){
      ROS_INFO_STREAM("A COLLISION WAS DETECTED");
      collision_detection::CollisionResult::ContactMap::const_iterator it;
      for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
        ROS_INFO("    Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }else
      ROS_INFO_STREAM("Position feasible");

    ros::WallDuration(4).sleep();
  }

  ros::shutdown();
  return 0;
}
