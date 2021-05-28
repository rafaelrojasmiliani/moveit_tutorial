#ifndef GRASP_TOOLS
#define GRASP_TOOLS 
//#include <moveit_grasps/grasp_planner.h>
//#include <moveit_grasps/suction_grasp_data.h>
//#include <moveit_grasps/suction_grasp_filter.h>
//#include <moveit_grasps/suction_grasp_generator.h>
#include <moveit_grasps/suction_grasp_scorer.h>

moveit_grasps::SuctionGraspScoreWeightsPtr
get_grasp_suction_score(double _x, double _y, double _z, double _raw,
                        double _pich, double _yaw, double _overhang);

moveit_grasps::SuctionGraspDataPtr get_grasp_data_initialized_from_parameters(
    const ros::NodeHandle &_nh, const std::string &_end_effector_name,
    const moveit::core::RobotModelConstPtr &_robot_model);

/*

bool get_feasible_grasp_poses(
    std::vector<moveit_grasps::GraspCandidatePtr> &_grasp_candidates,
    const std::string &_arm_name, const std::string &_object_name,
    moveit_grasps::SuctionGraspDataPtr _grasp_data,
    moveit_grasps::SuctionGraspScoreWeightsPtr _scores,
    std::vector<double> &_ideal_orientation);
*/
#endif /* ifndef  */
