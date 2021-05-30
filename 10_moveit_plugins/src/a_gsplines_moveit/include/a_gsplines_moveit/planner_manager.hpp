#ifndef PLANNER_MANAGER_H
#define PLANNER_MANAGER_H
#include <moveit/planning_interface/planning_interface.h>

namespace gsplines {

class GSplinesPlannerManager : public planning_interface::PlannerManager {
private:
  GSplinesPlannerManager(const GSplinesPlannerManager &that);
  GSplinesPlannerManager &operator=(const GSplinesPlannerManager &that);

public:
  GSplinesPlannerManager();
  virtual ~GSplinesPlannerManager();
  bool initialize(const moveit::core::RobotModelConstPtr &model,
                  const std::string &ns) override;
  bool
  canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const override;
  void getPlanningAlgorithms(std::vector<std::string> &algs) const override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr &planning_scene,
      const planning_interface::MotionPlanRequest &req,
      moveit_msgs::MoveItErrorCodes &error_code) const override;
};
} // namespace gsplines

#endif /* PLANNER_MANAGER_H */
