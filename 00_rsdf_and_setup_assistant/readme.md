
## Semantic Robot Description Format

The SRDF or Semantic Robot Description Format complement the URDF and specifies joint groups, default robot configurations, additional collision checking information, and additional transforms that may be needed to completely specify the robot’s pose. 
The recommended way to generate a SRDF is using the MoveIt Setup Assistant.

- **Self-Collisions** The Default Self-Collision Matrix Generator is part of the Setup Assistant. It searches for pairs of links on the robot that can safely be disabled from collision checking, decreasing motion planning processing time. These pairs of links are disabled when they are always in collision, never in collision, in collision in the robot’s default position or when the links are adjacent to each other on the kinematic chain. The sampling density specifies how many random robot positions to check for self collision. Higher densities require more computation time while lower densities have a higher possibility of disabling pairs that should not be disabled. The default value is 10,000 collision checks. Collision checking is done in parallel to decrease processing time.

- **Virtual Joints** are additional joints need to be defined to specify the pose of the root link of the robot with respect to a world coordinate system. E.g., a mobile robot is specified using a planar virtual joint that attaches the world coordinate frame to the frame of the robot. A fixed robot should be attached to the world using a fixed joint.

- **Passive Joints** are unactuated joints on a robot, e.g. passive casters on a differential drive robots. They are specified separately in the SRDF to make sure that different components in the motion planning or control pipelines know that the joints cannot be directly controlled.

- **Group** (sometimes called **JointGroup** or **Planning Group**) is a central concept in MoveIt. MoveIt will only consider moving the joints in the group that it is planning for and the other joints are left stationary. A group is simply a collection of joints and links. Each group can be specified in one of several different ways:
    - **Collection of Joints** All the child links of each joint are automatically included in the group.
    - **Collection of Links** All the parent joints of the links are also included in the group.
    - **Serial Chain**   serial chain is specified using the base link and the tip link. The tip link in a chain is the child link of the last joint in the chain. The base link in a chain is the parent link for the first joint in the chain.
Collection of Sub-Groups
    - **collection of groups**

- **End-Effectors** Certain groups in a robot can be given a special designation as an end-effector. An end-effector is typically connected to another group (like an arm) through a fixed joint. Note that when specifying groups that are end-effectors, it's important to make sure that there are no common links between the end-effector and the parent group it is connected to.


- **Robot Poses** The SRDF can also store fixed configurations of the robot. 

# MoveIt Setup Assistant


## MoveIt configuration package launch files

The name of the robot defined in the URDF is `myrobot`.

- `chomp_planning_pipeline.launch.xml`
- `default_warehouse_db.launch`
    - `warehouse.launch`
- `demo_gazebo.launch`
    - `gazebo.launch`
    - `planning_context.launch`
    - `move_group.launch`
    - `moveit_rviz.launch`
    - `default_warehouse_db.launch` if debug requested
- `demo.launch`
    - `planning_context.launch`
    - `move_group.launch`
    - `moveit_rviz.launch` if rviz requested
    - `default_warehouse_db.launch`  if debug requested
- `fake_moveit_controller_manager.launch.xml`
- `gazebo.launch`
    - `$(find gazebo_ros)/launch/empty_world.launch`
    - `ros_controllers.launch`
- `joystick_control.launch`
- `move_group.launch`
    - `planning_context.launch`
    - `planning_pipeline.launch.xml ns="move_group"` and `pipeline="ompl"`
    - `trajectory_execution.launch.xml if="$(arg allow_trajectory_execution)" ns="move_group"`
    - `sensor_manager.launch.xml if="$(ar allow_trajectory_execution)" ns="move_group"`
- `moveit.rviz`
- `moveit_rviz.launch`
- `myrobot_moveit_controller_manager.launch.xml`
- `myrobot_moveit_sensor_manager.launch.xml`
- `myrobot_planning_execution.launch`
    - `planning_context.launch`
    - `move_group.launch`
    - `moveit_rviz.launch`
- `ompl_planning_pipeline.launch.xml`
- `planning_context.launch`
- `planning_pipeline.launch.xml`
    - `$(arg pipeline)_planning_pipeline.launch.xml`
- `ros_controllers.launch`
- `run_benchmark_ompl.launch`
    - `planning_context.launch`
    - `warehouse.launch`
- `sensor_manager.launch.xml`
    - `$(arg moveit_sensor_manager)_moveit_sensor_manager.launch.xml` with `moveit_sensor_manager=myrobot`
- `setup_assistant.launch`
- `trajectory_execution.launch.xml`
    - `$(arg moveit_controller_manager)_moveit_controller_manager.launch.xml` with `moveit_controller_manager=myrobot`
- `warehouse.launch`
    - `warehouse_settings.launch.xml`
- `warehouse_settings.launch.xml`
