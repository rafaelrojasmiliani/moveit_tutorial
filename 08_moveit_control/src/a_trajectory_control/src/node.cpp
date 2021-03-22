
#include <boost/tokenizer.hpp>
#include <memory>
#include <moveit/macros/console_colors.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/node_name.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <set>
#include <tf2_ros/transform_listener.h>

namespace move_group {
class MyNode {
public:
  MyNode(const planning_scene_monitor::PlanningSceneMonitorPtr &psm, bool debug)
      : {

    context_.reset(new MoveGroupContext(psm, true, debug));

    configureCapabilities();
  }

  ~MyNode() {
    capabilities_.clear();
    context_.reset();
    capability_plugin_loader_.reset();
  }

  void status() { context_->status() }

private:
  void configureCapabilities() {
    try {
      capability_plugin_loader_.reset(
          new pluginlib::ClassLoader<MoveGroupCapability>(
              "moveit_ros_move_group", "move_group::MoveGroupCapability"));
    } catch (pluginlib::PluginlibException &ex) {
      ROS_FATAL_STREAM(
          "Exception while creating plugin loader for move_group capabilities: "
          << ex.what());
      return;
    }

    try {
      MoveGroupCapabilityPtr cap =
          capability_plugin_loader_->createUniqueInstance(
              "move_group/MoveGroupMoveAction");
      cap->setContext(context_);
      cap->initialize();
      capabilities_.push_back(cap);
    } catch (pluginlib::PluginlibException &ex) {
      ROS_ERROR_STREAM("Exception while loading move_group capability '"
                       << capability << "': " << ex.what());
    }
  }

  MoveGroupContextPtr context_;
  std::shared_ptr<pluginlib::ClassLoader<MoveGroupCapability>>
      capability_plugin_loader_;
  std::vector<MoveGroupCapabilityPtr> capabilities_;
};
} // namespace move_group

int main(int argc, char **argv) {
  ros::init(argc, argv, move_group::NODE_NAME);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));

  std::shared_ptr<tf2_ros::TransformListener> tfl =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
      new planning_scene_monitor::PlanningSceneMonitor("robot_description",
                                                       tf_buffer));

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor();
  planning_scene_monitor->startStateMonitor();

  move_group::MyNode mge(planning_scene_monitor, debug);

  mge.status();

  ros::waitForShutdown();

  return 0;
}
