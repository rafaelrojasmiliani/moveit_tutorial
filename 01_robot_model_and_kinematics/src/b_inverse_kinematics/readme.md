
- [kinematic base plugin](https://github.com/ros-planning/moveit/blob/f2cc2348de83557a5704cc0f8670413f37a7855d/moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.h#L146)

| Kinematic base method | type | use |
| --------------------  | ---- | --- |
| `getPositionIK`       | VIRTUAL | Given a desired pose of the end-effector, compute the joint angles to reach it |
| `searchPositionIK` (3 variations)   | VIRTUAL | Numerical method to search e.g. SVD  [here](https://github.com/ros-planning/moveit/blob/f2cc2348de83557a5704cc0f8670413f37a7855d/moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.h#L211)|
| `getPositionFK` | VIRTUAL | computes forward kinematics |
| `getJointNames` | virtual | Return all the joint names in the order they are used internally |
| `getLinkNames`  | virtual | Return all the joint names in the order they are used internally |



- [discretization error](https://github.com/ros-planning/moveit/blob/f2cc2348de83557a5704cc0f8670413f37a7855d/moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.h#L69)

- [kinematic error](https://github.com/ros-planning/moveit/blob/f2cc2348de83557a5704cc0f8670413f37a7855d/moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.h#L87)  

- [querry options](https://github.com/ros-planning/moveit/blob/f2cc2348de83557a5704cc0f8670413f37a7855d/moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.h#L109)

- [kineamtic result](https://github.com/ros-planning/moveit/blob/f2cc2348de83557a5704cc0f8670413f37a7855d/moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.h#L133)


## KDL kinematics plugins
It is declared [here](https://github.com/ros-planning/moveit/blob/f2cc2348de83557a5704cc0f8670413f37a7855d/moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h#L72) and defined [here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp).

The inverse kinematics is defined [here](https://github.com/ros-planning/moveit/blob/f2cc2348de83557a5704cc0f8670413f37a7855d/moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp#L316).
