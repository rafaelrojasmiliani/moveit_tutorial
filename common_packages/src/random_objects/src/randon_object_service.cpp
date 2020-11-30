

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string>

moveit_msgs::CollisionObject
random_object_msg(std::vector<moveit_msgs::CollisionObject> &object_list);

bool is_there_a_collision(planning_scene::PlanningScenePtr _ps);

class RandomObjectService {
public:
  RandomObjectService() : loader_("robot_description") {

    ros::NodeHandle nh;
    model_ = loader_.getModel();

    workspace_.reset(new planning_scene::PlanningScene(model_));

    service_ = nh.advertiseService(
        "random_object", &RandomObjectService::random_object_service, this);

    get_ps_srv_msg_client_ =
        nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

    set_ps_client_ = nh.serviceClient<moveit_msgs::ApplyPlanningScene>(
        "apply_planning_scene");
  }

  void spin() {
    get_ps_srv_msg_client_.waitForExistence();
    set_ps_client_.waitForExistence();
    ros::spin();
  }

  void random_object() {}
  bool random_object_service(std_srvs::TriggerRequest &request,
                             std_srvs::TriggerResponse &response) {

    // Creation of planning scene message
    moveit_msgs::ApplyPlanningScene set_ps_srv_msg;
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;
    moveit_msgs::GetPlanningScene get_ps_srv_msg;
    // 1. Get the current planning scene
    get_ps_srv_msg_client_.call(get_ps_srv_msg);
    // 2. Store the current planning scene in the local PlanningScene
    workspace_->usePlanningSceneMsg(get_ps_srv_msg.response.scene);
    // 2.1 Store the current objecst in a list
    std::vector<moveit_msgs::CollisionObject> &object_list =
        get_ps_srv_msg.response.scene.world.collision_objects;
    do {
      // 3. Generate a random object
      moveit_msgs::CollisionObject object = random_object_msg(object_list);
      // 4. Buil the PlanningScene message to add the object to the scene
      planning_scene_msg.world.collision_objects.push_back(object);
      // 5. Insert the new object to the local PlanningScene
      // 6. If there is a collission, remove the object
      workspace_->usePlanningSceneMsg(planning_scene_msg);
      if (not is_there_a_collision(workspace_)) {
        break;
      }
      // 6.1 change the operation type of the object to REMOVE
      planning_scene_msg.world.collision_objects[0].operation = object.REMOVE;
      // 6.2 Remove the object from the local PlanningScene
      workspace_->setPlanningSceneMsg(planning_scene_msg);
      // 6.3 remote the object from the local message
      planning_scene_msg.world.collision_objects.pop_back();
    } while (true);

    // apply the operation to the planning scene
    set_ps_srv_msg.request.scene = planning_scene_msg;
    bool srv_success = set_ps_client_.call(set_ps_srv_msg);
    return true;
  }
  virtual ~RandomObjectService() {}
  ros::ServiceServer service_;
  ros::ServiceClient get_ps_srv_msg_client_;
  ros::ServiceClient set_ps_client_;
  std::vector<std::string> object_list_;
  robot_model::RobotModelPtr model_;
  robot_model_loader::RobotModelLoader loader_;
  planning_scene::PlanningScenePtr workspace_;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "random_objects");
  std::srand(std::time(0));
  ROS_INFO("random objects .............................");

  RandomObjectService srv;
  ROS_INFO("random objects -----.............................");

  srv.spin();

  return 0;
}

bool is_there_a_collision(planning_scene::PlanningScenePtr _ps) {

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  _ps->checkCollision(collision_request, collision_result);
  if (collision_result.collision)
    return true;
  return false;
}
