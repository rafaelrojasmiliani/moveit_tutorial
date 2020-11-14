# What does this examples do?
This example teaches how `PlanningSceneMonitor` is implemented and how to use its ROS service API to add or remove objects.


## Files

| File | Use |
| ---- | --- |
|`srv/c_planning_scene_monitor_node.cpp` | Simple implementation of `PlanningSceneMonitor` |
|`src/c_planning_scene_api_node.cpp` | `PlanningSceneMonitor` ROS service API example and `movet_msg:PlanningScene`. |
|`src/solids.cpp` `include/solids.h` | Example of how to create shapes and `CollisionObject` |

## Planning Scene ROS API
Here we use the Planning Scene Monitor ROS service/topics API is to add or remove objects from the workspace.
These objects are divided in two main categories
1. Objects attached to the robot. These objects contains a variable which indicates the part of the robot that is in contact with the object. Then the collision detector will automatically ignore the contact between this object and the robot.
2. Objects not attached to the robot. Objects that are treated normally by the collision detector.

The main mechanism to attach, detach, add or remove objects from the workspace is to use the moveit message `moveit_msgs::PlanningScene` [defined here](http://docs.ros.org/en/jade/api/moveit_msgs/html/msg/PlanningScene.html).
Such a message type is very flexible and can be used define completely new workspaces.
The simplest way to apply modifications to a workspace already loaded in the `PlanningSceneMonitor` is through the scenario "diff" mechanism.
In such a method, we activate a field into the `moveit_msgs::PlanningScene` message which indicates that it indicates a modification to be applied to the current `planning_scene::PlanningScene` loaded into `PlanningSceneMonitor`.

In order to add or remove objects, the `moveit_msgs::PlanningScene` messages supports on the `moveit_msgs::CollisionObject` message to define objects and the `moveit_msgs::AttachedCollisionObject` to define objects attached to the robot.
However, nor `moveit_msgs::CollisionObject` nor `moveit_msgs::AttachedCollisionObject` are not direct subfields of `moveit_msgs::PlanningScene`.

1. **To add objects attached to the robot** we the required `moveit_msgs::PlanningScene::robot_state::attached_collision_object`.

2. **To add objects to the scenario** we add elements to `moveit_msgs::PlanningScene.world.collision_object`.

The `PlanningSceneMonitor` provides two interface to receive `moveit_msgs::PlanningScene` messages.
1. the service `"apply_planning_scene"` of type `moveit_msgs::ApplyPlanningScene` [defined here](http://docs.ros.org/en/jade/api/moveit_msgs/html/srv/ApplyPlanningScene.html) 
2. the topic `"planning_scene"` of type `moveit_msgs::PlanningScene` 


## `PlanningSceneMonitor`

`PlanningSceneMonitor` is [defined here](https://github.com/ros-planning/moveit/blob/47884198c2585215de8f365a7ff20479f8bb4b51/moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h#L61) and [implemented here](https://github.com/ros-planning/moveit/blob/47884198c2585215de8f365a7ff20479f8bb4b51/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp).
In its construction, this runs a `ros::AsyncSpinner`.

- **Function** `PlanningSceneMonitor::startSceneMonitor` by default it creates a subscriber to `planning_scene` with callback `PlanningSceneMonitor::newPlanningSceneCallback`

- **Function** `PlanningSceneMonitor::newPlanningSceneCallback`, just calls `PlanningSceneMonitor::newPlanningSceneMessage`

- **Function** `PlanningSceneMonitor::newPlanningSceneMessage`. Uses the tools from the package `moveit_ros_occupancy_map_monitor` and the class `occupancy_map_monitor::OccupancyMapMonitor` to update the scene.

- **Function** `PlanningSceneMonitor::startWorldGeometryMonitor` by default it creates a subscriber to `collision_object` with callback `PlanningSceneMonitor::collisionObjectCallback` and a subscriber to `planning_scene_world` with callback `PlanningSceneMonitor::newPlanningSceneWorldCallback`

- **Function** `PlanningSceneMonitor::collisionObjectCallback` 
- **Function** `PlanningSceneMonitor::newPlanningSceneWorldCallback` 




- **Subscribed topics**
    - `planning_scene` of type `moveit_msgs::PlanningScene` [defined here](http://docs.ros.org/en/api/moveit_msgs/html/msg/PlanningScene.html). Thus subscriber is instantiated at `PlanningSceneMonitor::startSceneMonitor`.
On this message the function `PlanningSceneMonitor::newPlanningSceneMessage` is called.
    - `collision_object` of type `moveit_msgs::CollisionObject` [defined here](http://docs.ros.org/en/jade/api/moveit_msgs/html/msg/CollisionObject.html) with callback `PlanningSceneMonitor::collisionObjectCallback`. This subscriber is instantiated at `PlanningSceneMonitor::startWorldGeometryMonitor`
    - `planning_scene_world` of type `moveit_msgs::PlanningScene` with callback `PlanningSceneMonitor::newPlanningSceneWorldCallback`.  This subscriber is instantiated at `PlanningSceneMonitor::startWorldGeometryMonitor`
    - `attached_collision_object` of type `moveit_msgs::AttachedCollisionObject` [defined here](http://docs.ros.org/en/jade/api/moveit_msgs/html/msg/AttachedCollisionObject.html) with callback `PlanningSceneMonitor::attachObjectCallback`.
    - `joint_states` subscribed by `PlanningSceneMonitor::current_state_monitor_` of type `CurrentStateMonitorPtr` with callback `planning_scene_monitor::CurrentStateMonitor::jointStateCallback` [implemented here](https://github.com/ros-planning/moveit/blob/382aa5a8cdd39eace07536d39c497a4b21f0f653/moveit_ros/planning/planning_scene_monitor/src/current_state_monitor.cpp#L336).

- **Published topics**
    - `monitored_planning_scene` of type `moveit_msgs::PlanningScene`.

- **Required services**
    - **Optional?** `get_planning_scene` of type `moveit_msgs::GetPlanningScene` [defined here](http://docs.ros.org/en/api/moveit_msgs/html/srv/GetPlanningScene.html) and [instantiated here](https://github.com/ros-planning/moveit/blob/a85738b7a1f1913bf97f99e138dfc8d22b0d3dad/moveit_ros/move_group/src/default_capabilities/get_planning_scene_service_capability.cpp#L48) by `move_group`
This service is called by `PlanningSceneMonitor::requestPlanningSceneState`

- **Offered services**
    - **Optional?** `get_planning_scene`with callback `PlanningSceneMonitor::getPlanningSceneServiceCallback` optional service for getting the complete planning scene.
This is useful for satisfying the Rviz PlanningScene display's need for a service **without having to use a `move_group` node**.
 _Be careful not to use this in conjunction with `PlanningSceneMonitor::requestPlanningSceneState`_, as it will create a pointless feedback loop
This service is initiated by `PlanningSceneMonitor::providePlanningSceneService`

    - `tf2_frames` through `PlanningSceneMonitor::tf_buffer_`

    - ` `
- **Threads**
    - `PlanningSceneMonitor::scenePublishingThread` [implemented here](https://github.com/ros-planning/moveit/blob/47884198c2585215de8f365a7ff20479f8bb4b51/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp#L334)

- **Members**
    - `planning_scene::PlanningScenePtr scene_;` Argument of the constructor, by default `planning_scene::PlanningScenePtr()`
    - `std::shared_ptr<tf2_ros::Buffer> tf_buffer_;` (this type is [defined here](https://github.com/ros/geometry2/blob/ad04943f23608ab757389ce57d04f110df1c692b/tf2_ros/include/tf2_ros/buffer.h#L51) and [implemented here](https://github.com/ros/geometry2/blob/ad04943f23608ab757389ce57d04f110df1c692b/tf2_ros/src/buffer.cpp)) Argument of the constructor, stores known frames.    `tf2_ros::TransformListener`
    - `std::unique_ptr<occupancy_map_monitor::OccupancyMapMonitor> octomap_monitor_;`
    - `robot_model_loader::RobotModelLoaderPtr rm_loader_;`, loads the model
    - `moveit::core::RobotModelConstPtr robot_model_;`
    - `collision_detection::CollisionPluginLoader collision_loader_;`
    - `DynamicReconfigureImpl* reconfigure_impl_;`

- **Parameters** defined using dynamic reconfigure [here](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_ros/planning/planning_scene_monitor/cfg/PlanningSceneMonitorDynamicReconfigure.cfg)
    - `"publish_planning_scene"` Set to True to publish Planning Scenes
    - `"publish_planning_scene_hz"` Set the maximum frequency at which planning scene updates are published
    - `"publish_geometry_updates"` Set to True to publish geometry updates of the planning scene
    - `"publish_state_updates"` Set to True to publish geometry updates of the planning scene
    - `"publish_transforms_updates"` Set to True to publish geometry updates of the planning scene


# Collision object 

`moveit_msgs::CollisionObject` [defined here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/CollisionObject.html)

- `header` of type `Header`, **`header.frame_id` required**
- `id` of type `string` **required** identifier name
- `type` of type `object_recognition_msgs/ObjectType` **required** The object type in a database of known objects
- `primitives` of type `shape_msgs/SolidPrimitive[]`  Solid geometric primitives
- `primitive_poses` of type `geometry_msgs/Pose[]` Their poses are with respect to the specified header
- `meshes` of type `shape_msgs/Mesh[]` Meshes
- `mesh_poses` of type `geometry_msgs/Pose[]` 
- `planes` of type `shape_msgs/Plane[]` Bounding planes (equation is specified, but the plane can be oriented using an additional pose)
- `plane_poses` of type `geometry_msgs/Pose[]` Bounding planes (equation is specified, but the plane can be oriented using an additional pose)
- `operation` of type `byte` Operation to be performed
    - `ADD=0` Adds the object to the planning scene. If the object previously existed, it is replaced.
    - `REMOVE=1` Removes the object from the environment entirely (everything that matches the specified id). If we generate a message witn remove and without objects ID, the Planning Scene Montiro will remove all objects [see here](https://github.com/ros-planning/moveit/blob/ff552bf861609f99ca97a7e173fcbeb0c03e9f45/moveit_core/planning_scene/src/planning_scene.cpp#L1812)
    - `APPEND=2` Append to an object that already exists in the planning scene. If the object does not exist, it is added.
    - `MOVE=3` If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)



# Attached Collision Object

`moveit_msgs::AttachedCollisionObject` [defined here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/AttachedCollisionObject.html)

- `link_name` of type `string`  The CollisionObject will be attached with a fixed joint to this link

- `object` of type `CollisionObject`  This contains the actual shapes and poses for the CollisionObject to be attached to the link If action is remove and no object.id is set, all objects  attached to the link indicated by link_name will be removed

- `touch_links` of type `string[]`  The set of links that the attached objects are allowed to touch  by default - the link_name is already considered by default

- `detach_posture` of type `trajectory_msgs/JointTrajectory`  If certain links were placed in a particular posture for this object to remain attached  (e.g., an end effector closing around an object), the posture necessary for releasing the object is stored here

- `weight` of type `float64`  The weight of the attached object, if known

# Planning Scene Message

- `name` of type `string` name of planning scene
- `robot_state` of type `RobotState` full robot state
- `robot_model_name` of type `string` The name of the robot model this scene is for
- `fixed_frame_transforms` of type `geometry_msgs/TransformStamped[]` additional frames for duplicating tf (with respect to the planning frame)
- `allowed_collision_matrix` of type `AllowedCollisionMatrix` full allowed collision matrix
- `link_padding` of type `LinkPadding[]` all link paddings
- `link_scale` of type `LinkScale[]` all link scales
- `object_colors` of type `ObjectColor[]` Attached objects, collision objects, even the octomap or collision map can have  colors associated to them. This array specifies them.
- `world` of type `PlanningSceneWorld` the collision map
- `is_diff` of type `bool` Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene

# How to attach objects to the robot

   We will use this message to add or subtract the object from the world and to attach the object to the robot.
 Since we are attaching the object to the robot hand to simulate picking up the object, we want the collision checker to ignore collisions between the object and the robot hand.

# How to add an object to the environment
   Add the object into the environment by adding it to the set of collision objects in the "world" part of the planning scene. Note that we are using only the "object" field of the `attached_object` message here.


