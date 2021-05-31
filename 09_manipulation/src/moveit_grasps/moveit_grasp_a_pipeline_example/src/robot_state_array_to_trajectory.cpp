
#include <moveit/robot_state/robot_state.h> // robot_state::RobotStatePtr
#include <moveit/robot_trajectory/robot_trajectory.h>

bool robot_state_array_to_trajectory(
    std::vector<robot_state::RobotStatePtr> &_rs_array,
    robot_trajectory::RobotTrajectory &_trajectory) {

  for (const robot_state::RobotStatePtr &rs : _rs_array) {
    _trajectory.addPrefixWayPoint(*rs, 0.0);
  }

  return true;
}
