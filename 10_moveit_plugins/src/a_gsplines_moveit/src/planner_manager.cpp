#include "a_gsplines_moveit/planner_manager.hpp"
namespace gsplines {

bool GSplinesPlannerManager::initialize(
    const moveit::core::RobotModelConstPtr &model,
    const std::string &ns) override {
  return true;
}

GSplinesPlannerManager::GSplinesPlannerManager() {}

GSplinesPlannerManager::GSplinesPlannerManager(
    const GSplinesPlannerManager &that) {}

GSplinesPlannerManager::~GSplinesPlannerManager() {}

planning_interface::PlanningContextPtr
GSplinesPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const planning_interface::MotionPlanRequest &req,
    moveit_msgs::MoveItErrorCodes &error_code) const override {
  return /* something */;
}

void GSplinesPlannerManager::getPlanningAlgorithms(
    std::vector<std::string> &algs) const override {}

} // namespace gsplines
