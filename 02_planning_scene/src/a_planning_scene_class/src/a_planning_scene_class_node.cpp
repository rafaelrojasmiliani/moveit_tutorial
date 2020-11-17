
/* Author: Rafael A. Rojas*/

#include <ros/ros.h> // ros::init, ros::AsyncSpinner,
#include <sensor_msgs/JointState.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/kinematic_constraints/utils.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a_planning_scene_class_example");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::size_t count = 0;
  ros::NodeHandle nh;

  ros::Publisher js_pub =
      nh.advertise<sensor_msgs::JointState>("joint_states_cmd", 1);
  sensor_msgs::JointState js;
  // 1) instantiate the  MoveIt robot model loader
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // 2) instantiate the  MoveIt PlanningScene with the moveit model
  robot_model::RobotModelPtr my_robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(my_robot_model);

  // 3) instantiate the input and output of the collision checking request
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_detection::CollisionResult collision_result;

  while (ros::ok()) {
    // 4. get the robot state and set the joints of the planning group to a
    // random position
    robot_state::RobotState &kinematic_model_actual =
        planning_scene.getCurrentStateNonConst();
    // 4.1 instantiate the planning group. The name myrobotplanninggroup was
    // defined with he moveit setup assistant
    const robot_model::JointModelGroup *joint_group =
        kinematic_model_actual.getJointModelGroup("myrobotplanninggroup");
    // 4.2 set the joint positions of the planning grou to a random position
    kinematic_model_actual.setToRandomPositions(joint_group);

    // 5. set the values of the planning request and the planning result
    collision_request.group_name = "myrobotplanninggroup";
    collision_result.clear();
    // 6. call the collision service (in this example this will call the FCL)
    planning_scene.checkSelfCollision(collision_request, collision_result);

    // 7. retrieve the random joint poisition and publish to the topic
    kinematic_model_actual.copyJointGroupPositions(joint_group, js.position);
    js.name = joint_group->getVariableNames();
    js_pub.publish(js);

    if (collision_result.collision) {
      ROS_INFO_STREAM("A COLLISION WAS DETECTED");
      collision_detection::CollisionResult::ContactMap::const_iterator it;
      for (it = collision_result.contacts.begin();
           it != collision_result.contacts.end(); ++it)
        ROS_INFO("    Contact between: %s and %s", it->first.first.c_str(),
                 it->first.second.c_str());
    } else
      ROS_INFO_STREAM("Position feasible");

    ros::WallDuration(4).sleep();
  }

  ros::shutdown();
  return 0;
}
