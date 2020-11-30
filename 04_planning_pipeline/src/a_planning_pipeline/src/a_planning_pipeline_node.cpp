/* Author: Rafael A. Rojas*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

// ROS MoveIt
#include <moveit/kinematic_constraints/utils.h> // kinematic_constraints::
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h> //robot_trajectory::
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>

// ROS Trajectory messages
#include <trajectory_msgs/JointTrajectoryPoint.h>

bool is_there_a_collision(planning_scene::PlanningScenePtr _ps);
void publish_trajectory(planning_interface::MotionPlanResponse &res);

class MotionPlanningService {
public:
  MotionPlanningService()
      : nh_(), loader_("robot_description"), model_(loader_.getModel()),
        workspace_(new planning_scene::PlanningScene(model_)),
        planning_pipeline_(model_, nh_, "planning_plugin", "request_adapters") {

    joint_state_pub_ =
        nh_.advertise<sensor_msgs::JointState>("joint_states_cmd", 10);

    robot_trajectory_pub_ =
        nh_.advertise<moveit_msgs::RobotTrajectory>("plot_trajectory", 10);

    service_ =
        nh_.advertiseService("plan_random_motion",
                             &MotionPlanningService::plan_random_motions, this);

    get_ps_client_ =
        nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  }

  void spin() {
    get_ps_client_.waitForExistence();
    ros::spin();
  }

  virtual ~MotionPlanningService() {}
  bool plan_random_motions(std_srvs::TriggerRequest &request,
                           std_srvs::TriggerResponse &response);
  void publish_trajectory(planning_interface::MotionPlanResponse &res);
  ros::NodeHandle nh_;
  ros::Publisher joint_state_pub_;
  ros::Publisher robot_trajectory_pub_;
  ros::ServiceServer service_;
  ros::ServiceClient get_ps_client_;
  robot_model_loader::RobotModelLoader loader_;
  robot_model::RobotModelPtr model_;
  planning_pipeline::PlanningPipeline planning_pipeline_;
  planning_scene::PlanningScenePtr workspace_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "plan_manager");

  MotionPlanningService mps;

  mps.spin();

  return 0;
}

bool MotionPlanningService::plan_random_motions(
    std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response) {

  moveit_msgs::GetPlanningScene
      get_ps_srv_msg;                        // used in 1. to get planning scene
  std::vector<double> start_joint_position;  // used in 3. to store joint pos.
  planning_interface::MotionPlanRequest req; // used in 6. to plan motion
  planning_interface::MotionPlanResponse res; // used in 6. to plan motion
  // 1. get the planning scene
  get_ps_client_.call(get_ps_srv_msg);
  workspace_->setPlanningSceneMsg(get_ps_srv_msg.response.scene);
  // 2. Get the joint group where the
  // plan will be done. NOTE: In this example there is only one joint group
  const robot_model::JointModelGroup *joint_group =
      model_->getJointModelGroup(model_->getJointModelGroupNames().back());
  // 3. Get the initial robot state and store it in a buffer
  robot_state::RobotState &state = workspace_->getCurrentStateNonConst();
  state.copyJointGroupPositions(joint_group, start_joint_position);
  // 4. compute a feasible random position using the robot_state already
  // retrieved
  do {
    state.setToRandomPositions(joint_group);
  } while (is_there_a_collision(workspace_));
  // 5. copy the feasible state and restore the state of the planning scene
  robot_state::RobotState goal_state(state);
  state.setJointGroupPositions(joint_group, start_joint_position);
  // 6. Plan a motion from the current state to the feasible random state
  moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_group);
  // 6.1 build MotionPlanRequest
  req.group_name = joint_group->getName();
  req.allowed_planning_time = 10.0;
  req.goal_constraints.push_back(goal_constraint);
  planning_pipeline_.generatePlan(workspace_, req, res);
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    ros::WallDuration(0.5).sleep();
    response.success = false;
    response.message = "Could not compute plan successfully";
    return false;
  }
  ROS_INFO("--------------------------------");
  ROS_INFO("Trajectory computed successfully");
  ROS_INFO("--------------------------------");

  publish_trajectory(res);

  response.success = true;
  response.message = "ok";
  return true;
}

bool is_there_a_collision(planning_scene::PlanningScenePtr _ps) {

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  _ps->checkCollision(collision_request, collision_result);
  if (collision_result.collision)
    return true;
  return false;
}

void MotionPlanningService::publish_trajectory(
    planning_interface::MotionPlanResponse &res) {

  static unsigned int seq = 1;
  sensor_msgs::JointState js;

  moveit_msgs::RobotTrajectory rtm; // robot trajectory message

  robot_trajectory::RobotTrajectoryPtr trajectory = res.trajectory_;
  trajectory->getRobotTrajectoryMsg(rtm);

  robot_trajectory_pub_.publish(rtm);

  js.name = trajectory->getGroup()->getVariableNames();
  const std::size_t n = trajectory->getWayPointCount();
  for (std::size_t i = 0; i < n; i++) {
    const robot_state::RobotState &state = trajectory->getWayPoint(i);
    state.copyJointGroupPositions(trajectory->getGroup(), js.position);
    js.header.seq = seq;
    ++seq;
    js.header.frame_id = "base_link";
    js.header.stamp = ros::Time::now();
    joint_state_pub_.publish(js);
    ros::WallDuration(0.2).sleep();
  }
}
