#include "simplemanipulator.hpp"
#include <string>

SimpleManipulator::SimpleManipulator(const std::string &_arm_group_name,
                                     const std::string &_ee_group_name)
    : robot_model_loader_("robot_description"),
      robot_model_(robot_model_loader_.getModel()),
      planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor(
          "robot_description")),
      visual_tools_(new moveit_visual_tools::MoveItVisualTools(
          robot_model_->getModelFrame(), "/rviz_visual_tools")),
      mgi_(_arm_group_name), psi_(),
      arm_jmg_(robot_model_->getJointModelGroup(_arm_group_name)),
      ee_jmg_(robot_model_->getJointModelGroup(_ee_group_name)),
      arm_flange_link_name_(ee_jmg_->getEndEffectorParentGroup().second) {}

SimpleManipulator::~SimpleManipulator() {}
