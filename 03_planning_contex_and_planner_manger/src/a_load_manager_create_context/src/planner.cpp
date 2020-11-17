/* Author: Rafael A. Rojas*/

#include <pluginlib/class_loader.h> // pluginlib::ClassLoader
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h> // kinematic_constraints::
#include <moveit/planning_interface/planning_interface.h> // planning_interface::PlannerManagerPtr
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h> //robot_trajectory::
#include <moveit_msgs/GetPlanningScene.h>

// Trajectory messages
#include <trajectory_msgs/JointTrajectoryPoint.h>
// C++
#include <boost/scoped_ptr.hpp> //boost::scoped_ptr

// loads the plan manager plugin given a parameter name
planning_interface::PlannerManagerPtr
load_plan_manager_plugin(const std::string _param_name);
// evaluates if the planning_scene has a colision in its current state
bool is_there_a_collision(planning_scene::PlanningScenePtr _ps);
// loop to plan random motions
void plan_random_motions(robot_model::RobotModelPtr _model,
                         planning_interface::PlannerManagerPtr _plan_manager);
// instantiates a planning context with the input data and
// tries to generate a plan to reach _state
planning_interface::MotionPlanResponse
plan_group_to_state(planning_scene::PlanningScenePtr _ps,
                    planning_interface::PlannerManagerPtr _plan_manager,
                    const robot_model::JointModelGroup *_joint_group,
                    robot_state::RobotState &_state);
// publish the trajectory to joint_states_cmd
void publish_trajectory(planning_interface::MotionPlanResponse &res);
int main(int argc, char **argv) {
  ros::init(argc, argv, "plan_manager");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  robot_model_loader::RobotModelLoader loader("robot_description");
  robot_model::RobotModelPtr model = loader.getModel();

  planning_interface::PlannerManagerPtr plan_manager =
      load_plan_manager_plugin("planning_plugin");

  if (not plan_manager) {
    ROS_FATAL_STREAM("Failted to load plugin");
    ros::shutdown();
    return 0;
  }

  bool can_initialize =
      plan_manager->initialize(model, node_handle.getNamespace());

  if (not can_initialize) {
    ROS_FATAL_STREAM("Failed to initialize plugin");
    ros::shutdown();
    return 0;
  }

  plan_random_motions(model, plan_manager);

  ros::waitForShutdown();
  return 0;
}

planning_interface::PlannerManagerPtr
load_plan_manager_plugin(const std::string _param_name) {

  std::string planner_plugin_name;

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>>
      planner_plugin_loader;
  planning_interface::PlannerManagerPtr plan_manager;
  ros::NodeHandle node_handle;

  if (!node_handle.getParam(_param_name, planner_plugin_name)) {
    ROS_FATAL_STREAM("Could not find planner plugin name");
    return 0;
  }

  planner_plugin_loader.reset(
      new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
  plan_manager.reset(
      planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));

  return plan_manager;
}

void plan_random_motions(robot_model::RobotModelPtr _model,
                         planning_interface::PlannerManagerPtr _plan_manager) {
  /*This functions will request for plans to move the robot from its actual
   * state to a random position in the joint space.*/
  // 1. initializes ros stuff
  ros::NodeHandle node_handle;
  // 2. instantiates a planning scene using the robot moveit model
  // NOTE: here we use a pointer because planning context requres a pointer
  planning_scene::PlanningScenePtr workspace(
      new planning_scene::PlanningScene(_model));

  // 3. Creates a client to update the planning scene
  ros::ServiceClient client =
      node_handle.serviceClient<moveit_msgs::GetPlanningScene>(
          "get_planning_scene");

  client.waitForExistence();
  moveit_msgs::GetPlanningScene srv;
  // 4. Get the group name and instantes a point to the joint group where the
  // plan will be done. NOTE: In this example there is only one joint group
  const std::string joint_group_name = _model->getJointModelGroupNames().back();
  const robot_model::JointModelGroup *joint_group =
      _model->getJointModelGroup(joint_group_name);
  while (ros::ok()) {
    // 5. retrieve the planning scene
    client.call(srv);
    workspace->setPlanningSceneMsg(srv.response.scene);
    // 6. Get the initial robot state and store it in a buffer
    robot_state::RobotState &state = workspace->getCurrentStateNonConst();
    std::vector<double> start_joint_position;
    state.copyJointGroupPositions(joint_group, start_joint_position);
    // 7. compute a feasible random position using the robot_state already
    // retrieved
    do {
      state.setToRandomPositions(joint_group);
    } while (is_there_a_collision(workspace));
    // 8. copy the feasible state and restore the state of the planning scene
    robot_state::RobotState goal_state(state);
    state.setJointGroupPositions(joint_group, start_joint_position);
    // 9. Plan a motion from the current state to the feasible random state
    planning_interface::MotionPlanResponse result;
    result =
        plan_group_to_state(workspace, _plan_manager, joint_group, goal_state);
    if (result.error_code_.val != result.error_code_.SUCCESS) {
      ROS_ERROR("Could not compute plan successfully");
      ros::WallDuration(0.5).sleep();
    } else {
      ROS_INFO("-----------------------\n Trajectory computed successfully\n "
               "----------------------------");
      publish_trajectory(result);
    }
  }
}

bool is_there_a_collision(planning_scene::PlanningScenePtr _ps) {

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  _ps->checkCollision(collision_request, collision_result);
  if (collision_result.collision)
    return true;
  return false;
}

planning_interface::MotionPlanResponse
plan_group_to_state(planning_scene::PlanningScenePtr _ps,
                    planning_interface::PlannerManagerPtr _plan_manager,
                    const robot_model::JointModelGroup *_joint_group,
                    robot_state::RobotState &_state) {

  moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(_state, _joint_group);
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse result;

  req.group_name = _joint_group->getName();
  req.allowed_planning_time = 1.0;
  req.goal_constraints.push_back(goal_constraint);
  planning_interface::PlanningContextPtr context =
      _plan_manager->getPlanningContext(_ps, req, result.error_code_);
  context->solve(result);
  return result;
}

static unsigned int seq = 1;
void publish_trajectory(planning_interface::MotionPlanResponse &res) {

  ros::NodeHandle node_handle;
  ros::Publisher js_pub =
      node_handle.advertise<sensor_msgs::JointState>("joint_states_cmd", 10);
  sensor_msgs::JointState js;

  robot_trajectory::RobotTrajectoryPtr trajectory = res.trajectory_;
  js.name = trajectory->getGroup()->getVariableNames();
  const std::size_t n = trajectory->getWayPointCount();
  for (std::size_t i = 0; i < n; i++) {
    const robot_state::RobotState &state = trajectory->getWayPoint(i);
    state.copyJointGroupPositions(trajectory->getGroup(), js.position);
    js.header.seq = seq;
    ++seq;
    js.header.frame_id = "base_link";
    js.header.stamp = ros::Time::now();
    js_pub.publish(js);
    ros::WallDuration(0.1).sleep();
  }
}
