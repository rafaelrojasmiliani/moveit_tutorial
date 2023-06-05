# Collision Detection

The moveit Front-end of collision detection is the plannins scene monitor, which is a ros grapper of the planning scene.
However, plannins scene is a wrapper of other tools which handle collision and implement the [Collision Detection Plugin](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_plugin.h#L80).

## The Plugin

The [`collision_detection::CollisionPlugin`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_plugin.h#L80) has only one pure virtual method which takes a planning scene as argument.
In synthesis it is a wrapper of a custom [`collision_detection::CollisionEnv`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_env.h#LL51C7-L51C19) which implements the collision detection.

This is the entry point of the api to interact with other plugins. As described in [this example](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_plugin.h#L46) the intended way to implement a collision detection is combining a `collision_detection::CollisionPlugin` with a [`collision_detection::CollisionDetectorAllocator`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_detector_allocator.h#L47) and a [`collision_detection::CollisionEnv`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_env.h#LL51C7-L51C19) via the [`CollisionDetectorAllocatorTemplate`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_detector_allocator.h#LL72C7-L72C41) as inheritance helper.
Then we have to invoche [`collision_detection::CollisionDetectorAllocatorTemplate::create`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_detector_allocator.h#L91) to get a `CollisionDetectorAllocator` which has the method `allocateEnv` to allocate new `collision_detection::CollisionEnv`.

[`CollisionDetectorFCLPluginLoader` (the actual plugin)](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection_fcl/include/moveit/collision_detection_fcl/collision_detector_fcl_plugin_loader.h#L44) with its [initializer](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection_fcl/src/collision_detector_fcl_plugin_loader.cpp#L42). [`CollisionDetectorAllocatorFCL`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection_fcl/include/moveit/collision_detection_fcl/collision_detector_allocator_fcl.h#LL45C7-L45C36) and [`CollisionEnvFCL`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection_fcl/include/moveit/collision_detection_fcl/collision_env_fcl.h#L53).


## The Plannins Scene default plugin and collision checking interface

With ros noetic, the planning scene initializes the collision detection [here](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/planning_scene/src/planning_scene.cpp#L156) which is called in its construction.
This takes [here](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/planning_scene/src/planning_scene.cpp#L284) where `PlanningScene::active_collision_` and `PlanningScene::collision_` variables are updated.

The method of the planning scene to check collisions are [`checkCollision`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/planning_scene/src/planning_scene.cpp#L462) and [`checkSelfCollision`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/planning_scene/src/planning_scene.cpp#L478) which wraps calls to [`PlanningScene::active_collision_->getCollisionEnv()->checkRobotCollision(...)`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_env.h#L120)
The collision detector is wrasped in [`collision_detection::CollisionDetector`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h#L1084) (which grasp [`collision_detection::CollisionEnv`](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_env.h#L51)).

## The Collision enviroment

The `collision_detection::CollisionEnv` [declared here](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/include/moveit/collision_detection/collision_env.h#LL51C13-L51C13) and [defined here](https://github.com/ros-planning/moveit/blob/4aeccc712293577e64918c0bb185ef8c38eeed84/moveit_core/collision_detection/src/collision_env.cpp#L72) is the main interface for Collisions Plugins.


# World and opjbect

[`collision_detection::Object`](https://github.com/ros-planning/moveit/blob/5b430a3d66aec0543d77a0963b0e3b537c4a42be/moveit_core/collision_detection/include/moveit/collision_detection/world.h#L79) is a struct that contains [shapes](https://github.com/ros-planning/geometric_shapes/blob/e7025cfc6c97bf1803463067d772b2658b9948de/include/geometric_shapes/shapes.h#L77) in an array [here](https://github.com/ros-planning/moveit/blob/5b430a3d66aec0543d77a0963b0e3b537c4a42be/moveit_core/collision_detection/include/moveit/collision_detection/world.h#L97) and their position (relative and absolute).

Objects are contained in `collision_detection::World` as a map [`collision_detection::World::objects_`](https://github.com/ros-planning/moveit/blob/5b430a3d66aec0543d77a0963b0e3b537c4a42be/moveit_core/collision_detection/include/moveit/collision_detection/world.h#L337).

## Octomap

- **The id of the octomap in the collision matrix is** `<octomap>`.
The ros API front end of octomaps is the planning scene monitor.
the PSM listest for octomap updated here and binds it to the plannins scene [here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp#LL1251C15-L1251C32) by a call to [`PlanningScene::processOctomapPtr`](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_core/planning_scene/src/planning_scene.cpp#L1436).
