# MoveIt visual tools

The 

```C++
  MoveItVisualTools(const std::string& base_frame, const std::string& marker_topic,
                    planning_scene_monitor::PlanningSceneMonitorPtr psm);

  MoveItVisualTools(const std::string& base_frame,
                    const std::string& marker_topic = rviz_visual_tools::RVIZ_MARKER_TOPIC,
                    robot_model::RobotModelConstPtr robot_model = robot_model::RobotModelConstPtr());
```
The marker topic is [defined here](https://github.com/PickNikRobotics/rviz_visual_tools/blob/1a1a4d153acdb465606d4c058cb91dcffdd1eb28/include/rviz_visual_tools/rviz_visual_tools.h#L86).
