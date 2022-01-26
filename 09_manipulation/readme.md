# Moveit Manipulation

Moveit provides three main methods to manipulate objects.

- [pick place](https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html)
- [moveit grasp](https://ros-planning.github.io/moveit_tutorials/doc/moveit_grasps/moveit_grasps_tutorial.html)
- [deep grasp](https://ros-planning.github.io/moveit_tutorials/doc/moveit_deep_grasps/moveit_deep_grasps_tutorial.html)

In order to present pick and place operation, the following messages, services and actions are defined

- **Message types**
    - [Grasp](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/Grasp.html)
    - [PlaceLocation](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/PlaceLocation.html)
    - [GripperTranslation](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/GripperTranslation.html)

- **Service types**
    - [GraspPlanning](http://docs.ros.org/en/noetic/api/moveit_msgs/html/srv/GraspPlanning.html)

- **Actions**
    - [Pickup](http://docs.ros.org/en/noetic/api/moveit_msgs/html/action/Pickup.html)
    - [Place](http://docs.ros.org/en/noetic/api/moveit_msgs/html/action/Place.html)


## Considerations

In the moveit context, an end-effector is a particular kind of `JointModelGroup` [see here](https://github.com/ros-planning/moveit/blob/55aee7130851654fa198745649465a4a1c386fc4/moveit_core/robot_model/include/moveit/robot_model/joint_model_group.h#L68).
For this reason, in order to properly plan a grasp, we require to handle at least two different joint groups, e.g. the arm and the gripper.
Then, a grasp plan require to compute a plan  for the arm where the gripper is mounted and for the gripper.
The plan request for the arm is given in terms of the link of the arm where the end-effector is mounted.

## Grasp message

The [Grasp message](http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/Grasp.html) contains a description of a grasp that would be used with a particular end-effector to grasp an object.
It does not contains any grasp point as a position ON the object, but contains the grasp pose of the arm where the end-effector is mounted.
This is done to decouple the arm-related planning from the gripper.
This information includes,
1. How to approach the object
3. How to set the gripper before gripping
4. How to grip it
5. How to retread after the object was grasp.

| Message member | Meaning |
| -------------- | ------- |
| `string id`  | an id, a name for the grasp.|
| `trajectory_msgs/JointTrajectory pre_grasp_posture` | This is used [here](https://github.com/ros|planning/moveit_tutorials/blob/9e2622861cf9e4373b93169a4a7bb853ed3b04d3/doc/pick_place/src/pick_place_tutorial.cpp#L124) to set the joints positions of the hand (only define these joints) that represent the hand opened. It is described as "The internal posture of the hand for the pre|grasp. Only positions are used"|
| `trajectory_msgs/JointTrajectory grasp_posture` |Is used [here](https://github.com/ros|planning/moveit_tutorials/blob/9e2622861cf9e4373b93169a4a7bb853ed3b04d3/doc/pick_place/src/pick_place_tutorial.cpp#L130) to specify the join configuration that closes the gripper. It is described as "The internal posture of the hand for the grasp. Only positions and efforts are used"|
| `geometry_msgs/PoseStamped grasp_pose` | This is the pose of the parent link of the end|effector, **not actually the pose of any link in the end|effector**.|
| `float64 grasp_quality` | The estimated probability of success for this grasp, or some other measure of how "good" it is.|
| `GripperTranslation pre_grasp_approach` | approach direction to take before picking an object|
| `GripperTranslation post_grasp_retreat` | The retreat direction to take after a grasp has been completed (object is attached)|
| `GripperTranslation post_place_retreat` | The retreat motion to perform when releasing the object;|
| `float32 max_contact_force` | the maximum contact force to use while grasping|
| `string[] allowed_touch_objects` | an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping|

## GripperTranslation

The `GripperTranslation` message defines a translation for the gripper, used in pickup or place tasks  for example for lifting an object off a table or approaching the table for placing

- `geometry_msgs/Vector3Stamped direction` the direction of the translation
- `float32 desired_distance` the desired translation distance
- `float32 min_distance` the min distance that must be considered feasible before the grasp is even attempted

## The grasp/place pipeline

1. Generation of grasp candidate
2. Find the feasible grasp candidates (filtering)
3. Plan motion



# The compute cartesian path function
