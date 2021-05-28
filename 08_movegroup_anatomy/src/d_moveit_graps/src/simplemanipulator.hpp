#ifndef SIMPLEMANIPULATOR_H
#define SIMPLEMANIPULATOR_H

class SimpleManipulator {
private:
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  const moveit::core::JointModelGroup *arm_jmg_;
  const moveit::core::JointModelGroup *ee_jmg_;
  const robot_model::RobotModelPtr &robot_model_;
  moveit::planning_interface::MoveGroupInterface mgi_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  const robot_model::LinkModel *arm_flange_link_;
  //   parent_link_ =
  //   robot_model_->getLinkModel(ee_jmg_->getEndEffectorParentGroup().second);
  //

public:
  SimpleManipulator(const std::string &_group_name);
  SimpleManipulator(const SimpleManipulator &that);
  SimpleManipulator &operator=(const SimpleManipulator &);
  virtual ~SimpleManipulator();
};

#endif /* SIMPLEMANIPULATOR_H */
