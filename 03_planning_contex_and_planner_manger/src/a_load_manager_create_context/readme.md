**Note: work in progress**
# What does this example do?

This example teach how to plan motions in the joint space using the moveit plugin for OMPL.

## Planner

In this instance we learn the basics of motion planning

1. Load the planner manager

2. 


| File | Description |
| ---- | ----------- |
| `src/planning_scene_monitor.cpp` | Instance of a Planning Scene Monitor. |
| `src/planner.cpp` | Implementation of the motion planning serice with moveit OMPL plugin. |
| `src/orchestrator.cpp` | Adds/removes random objects as class the motion planning service. |
| `src/trajectory_plot.py` | Plots the output of the motion planning|
| `launch/planner.launch` | Launch the example|
| `launch/planning_scene_monitor.launch` | Launch the Planning Scene Monitor|



```mermaid
graph TD;
    subgraph Planner
    subgraph loop
    RPS[Request actual<br/>planning scene] --> GRP[Get random joint pose];
    GRP --> CC{collision?};
    CC -- NO --> CMP[Compute Motion Plan];
    CC -- YES --> GRP;
    end
    RML[Robot Model Loader] -- instantiates --> RM[Robot Model];
    MPP[pluginlib::ClassLoader<br/>planning_interface::PlannerManager] -- instantiates --> PM[PlannerManager];
    PPP[ROS param <br/>planning_plugin] -- constructor<br/>argument --> PM;
    RM -- initialize method --> PM;
    RM -- contains --> MG[Move Group];
    PM -- instantiates --> PC[PlanningContext];
    MPR[MotionPlanRequest] -- instantiation arg. --> PC;
    PS -- instantiation arg. --> PC;
    PC -- solve --> MPRES[MotionPlanResponse];
    PS[`PlanningScene`] -- contains --> RM;
    RM -- used to instantiate --> PS;
    PS -- contains --> RS[Robot State Representation];
    PS -- contains --> CD[Collision detection interface];
    RS -- manipulates --> MG;
    end
    subgraph PSM
    RML2[Robot Model Loader] -- instantiates --> PSM[Planning Scene Monitor];
    PSM -- instantiates --> RM2[Robot Model];
    PSM -- expose --> APSS[apply_planning_scene service]
    PSM -- expose --> GPSS[get_planning_scene service]
    PSM -- Publish --> MPS[monitored_planning_scene]
    end
    JS[JointState] -- subscribes --> PSM;
    style PS fill:#CFFFCD;
    style RS fill:#CFFFCD;
    style CD fill:#CFFFCD;
    style PM fill:#FFD2D2;
    style PC fill:#FFD2D2;
    style MPR fill:#FFD2D2;
    style MPRES fill:#FFD2D2;
```

# How did we create this package?

1. Create the catkin package
```CMake
catkin create pkg PROJECT_NAME --catkin-deps roscpp moveit_core moveit_ros_planning_interface --system-deps Eigen3
```

2. In `CMakeLists.txt` add the executable indicated name of the executable and source file with `main`
```CMake
add_executable(${PROJECT_NAME}_node src/SOURCE.cpp)
```
3. In `CMakeLists.txt` add the libraries to link against
```CMake
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
```
## Current state monitor
This is an instance of Plannisn Scene motnior [defined here](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/current_state_monitor.h) and [implemented here](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_ros/planning/planning_scene_monitor/src/current_state_monitor.cpp).

## Required paramenters

In order to load the motion planner plugin this examples as the moveit configuration package `move_group` node requires a parameter with the name of the plugin.
- `planning_plugin`  e.g. `ompl_interface/OMPLPlanner`
```XML
<param name="planning_plugin" value="ompl_interface/OMPLPlanner" />
```

## Defining constraints
    
    Serveral tools to define constraints are written in [`moveit/moveit_core/kinematic_constraints/src/utils.cpp`](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_core/kinematic_constraints/src/utils.cpp).

## `moveit_msg::JointConstraints`

- `joint_name`of type `string`
- `position`of type `float64`
- `tolerance_above`of type `float64`
- `tolerance_below`of type `float64`
- `weight`of type `float64`

## `moveit_msg::PositionConstraint`

- `header` of type `Header`
- `link_name`of type `string` The robot link this constraint refers to
- `target_point_offset`of type `geometry_msgs/Vector3` The offset (in the link frame) for the target point on the link we are planning for
- `constraint_region`of type `BoundingVolume` The volume this constraint refers to 
- `weight`of type `float64` A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)

## `moveit_msg::BoundingVolume`

- `primitives`of type `shape_msgs/SolidPrimitive[]` A set of solid geometric primitives that make up the volume to define (as a union)
- `primitive_poses`of type `geometry_msgs/Pose[]` The poses at which the primitives are located
- `meshes`of type `shape_msgs/Mesh[]` In addition to primitives, meshes can be specified to add to the bounding volume (again, as union)
- `mesh_poses`of type `geometry_msgs/Pose[]` The poses at which the meshes are located

