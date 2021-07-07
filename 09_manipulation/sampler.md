

The first pick position is found in the "reachable valed pose filter" stage of the pick planning pipeline.
This position is sampled from a constrained sampler, which saples joint positions from the constraint generated as
```C++
    plan->goal_constraints_ =
        kinematic_constraints::constructGoalConstraints(plan->shared_data_->ik_link_->getName(), plan->goal_pose_);
    plan->goal_sampler_ =
        constraints_sampler_manager_->selectSampler(planning_scene_, planning_group, plan->goal_constraints_);
```

This piece of code is [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L138) where the sampler is appened to the plan [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L140).


The constrained sampler manger loader in `PickPlace` first instantiated [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_ros/manipulation/pick_place/src/pick_place.cpp#L95).
Then `PickPlace` makes available the sampler manager [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L126) by calling `ConstraintSamplerManagerLoader::getConstraintSamplerManager`.

## The Constrain samples.

The constrait samples is implemented in `moveit_core`, is declared [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler.h#L59) and defined [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/src/constraint_sampler.cpp#L39).
Hoewever, MoveIt provides an infrastructure to handle constraind samplers.


| Class | Purpose | Declared | Implemented |
| ----- | ------  | -------- | ----------- |
| `ConstraintSamplerAllocator` | ? | [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_allocator.h#L46) | PURE VIRTUAL |
| `ConstraintSamplerManager` | ssists in the generation of a `ConstraintSampler` for a  particular group from a `moveit_msgs::Constraints` | [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_manager.h#L56) | [here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_core/constraint_samplers/src/constraint_sampler_manager.cpp) |
| `ConstraintSampler` | abstract base class that allows the sampling of a kinematic state for a particular group of a robot | [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler.h#L59) | PURE VIRTUAL [partially here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_core/constraint_samplers/src/constraint_sampler.cpp) |
| `JointConstraintSampler` | Default implementation of `ConstraintSampler` | [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.h#L56) | [here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_core/constraint_samplers/src/default_constraint_samplers.cpp) |
| `UnionConstraintSampler` | Union of `ConstraintSampler` | [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/union_constraint_sampler.h#L56) | [here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_core/constraint_samplers/src/union_constraint_sampler.cpp) |


## Constraint Sampler Manager

This managers is a container of `ConstraintSamplerAllocator`, which are stored in a vector of pointers called `sampler_alloc_`.
To retrive these The function  `ConstraintSamplerManager::selectSampler` declared [here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_manager.h#L91) and [defined here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/src/constraint_sampler_manager.cpp#L43)

This function will iterate through the constraint sampler allocators, trying to find one that can service the constraints.
The first one that can service the request will be called.  If no allocators can service the Constraints, or there are no  allocators, the selectDefaultSampler will be called.

If there are not saplers, `selectSampler` will return the "default sampler" returned by `constraint_samplers::ConstraintSamplerManager::selectDefaultSampler` [defined here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/src/constraint_sampler_manager.cpp#L56).


### Default Samplers

- `JointConstraintSampler` [declared here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.h#L56)

- `IKSamplingPose` [declared here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.h#L209)

- `IKConstraintSampler` [declared here](https://github.com/ros-planning/moveit/blob/a2be2d8f569399123ac84d71279bbd143f49d17e/moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.h#L292)
  Allows the sampling joints positions with constraints in Cartesian space..
 Attempts to sample a pose that adheres to the constraint, then solves IK for that pose.


Default logic to select a ConstraintSampler given a constraints message.

This function will generate a sampler using the `joint_constraint,` `position_constraint` and `orientation_constraint` vectors from
the `moveit_msgs::Constraints` argument.
The type of constraint sampler that is produced depends on which constraint vectors have been
                        populated.

    The following rules are applied:

- If every joint in the group indicated by `group_name` is constrained by a
   JointConstraintSampler is returned.

- If thera are not valid constraints a JointConstraintSampler with unbounded joints is returned.

- If position and orientation constraints are present and there is an IKSolver
   for the group, the function will attempt to create an IKConstraintSampler.
    - If there are multiple valid position/constraint pairings, the one with the
     smallest volume will be kept.
    - If no full pose is available, the function will attempt to create a
     position-only IKConstraintSampler.
   - Finally, the function will attempt to create an orientation-only
     IKConstraintSampler.
   - If there is a valid IKConstraintSampler, then if no valid joint
     constraints are present then an *IKConstraintSampler will be returned.
   - If there are joint constraints, a UnionConstraintSampler with both the
     JointConstraintSampler and the *IKConstraintSampler will be returned.
 - If there is no direct IK solver for the group, or no valid
   IKConstraintSampler could be generated, and there are *subgroup IKSolvers,
   the function will attempt to generate a sampler from the various subgroup
   solvers.
   - It will attempt to determine which constraints act on the IK link for the
     sub-group IK solvers, and attempts to *create ConstraintSampler functions
     by recursively calling \ref selectDefaultSampler for the sub-group.
   - If any samplers are valid, it adds them to a vector of type \ref
     ConstraintSamplerPtr.
   - Once it has iterated through each sub-group, if any samplers are valid,
     they are returned in a *UnionConstraintSampler, along with a
     JointConstraintSampler if one exists.  @param scene The planning scene
     that will be used to create the ConstraintSampler @param group_name The
     group name for which to create a sampler @param constr The set of
     constraints for which to create a sampler

 @return A valid \ref ConstraintSamplerPtr if one could be allocated, and
  otherwise an empty \ref *ConstraintSamplerPtr
                                                                                                                               */


## Ros interface

The ROS interface of the constrain samples is implemented in `moveit_ros/planning/constraint_sampler_manager_loader`.
This interface consist only in `ConstraintSamplerManagerLoader` [declared here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_ros/planning/constraint_sampler_manager_loader/include/moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h) and [defined here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_ros/planning/constraint_sampler_manager_loader/src/constraint_sampler_manager_loader.cpp).
This class is a wrapper for a pointer to `constraint_samplers::ConstraintSamplerManager` which by default is instantiate with the default constructor of `ConstraintSamplerManager`.


## `ConstraintSamplerManager::selectSampler`
