
# Creating new setup assistant

## Loading and parsing URDF XACRO

The setup-assistant implements a methodology to parse and load XACRO files. The investigation of its nature is described here.
1. URDF/XACRO files are loaded [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/start_screen_widget.cpp#L261)
2. then [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/start_screen_widget.cpp#L465)
3. then [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/start_screen_widget.cpp#L547)
4. **Conclusion** To load the URDF/XACRO file they use `rdf_loader::RDFLoader::loadXmlFileToString` [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/start_screen_widget.cpp#L549). They check if a file is xacro with `rdf_loader::RDFLoader::isXacroFile`
5. then [here](https://github.com/ros-planning/moveit/blob/47aa486c0522feb91f93478da77f20bdd5e59983/moveit_ros/planning/rdf_loader/src/rdf_loader.cpp#L2019)
6. then check if it is a XACRO fle [here](https://github.com/ros-planning/moveit/blob/47aa486c0522feb91f93478da77f20bdd5e59983/moveit_ros/planning/rdf_loader/src/rdf_loader.cpp#L123)
7. **Conclusion** the setup assistant loads a xacro with POPEN an calling the python sub-process [here](https://github.com/ros-planning/moveit/blob/47aa486c0522feb91f93478da77f20bdd5e59983/moveit_ros/planning/rdf_loader/src/rdf_loader.cpp#L161)


## Generatng a SRDF from the URDF

1. A minimal srdf file is created [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/start_screen_widget.cpp#L507)
2. a introduced [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/start_screen_widget.cpp#L609) into the `config_data_`
3. **Conclusion** The minimal empty srdf and the urdf are used to generate a robot model in `MoveItConfigFata` [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/tools/moveit_config_data.cpp#L96)

```cpp
  // get the path from the urdf file loader QT widget
  config_data_->urdf_path_ = urdf_file_->getPath();

  // Check that this file exits
  if (!fs::is_regular_file(config_data_->urdf_path_))
    return false;

  // Attempt to get the ROS package from the path where the URDF is used
  // this looks for a "package.xml"
  if (!extractPackageNameFromPath())
    return false;  // An error occurred

  // Progress Indicator
  progress_bar_->setValue(20);
  QApplication::processEvents();

  // use xacro args from GUI
  config_data_->xacro_args_ = urdf_file_->getArgs().toStdString();

  // Load the URDF to the parameter server and check that it is correct format
  if (!loadURDFFile(config_data_->urdf_path_, config_data_->xacro_args_))
    return false;  // error occurred

  // Progress Indicator
  progress_bar_->setValue(50);
  QApplication::processEvents();

  // Create blank SRDF file
  const std::string blank_srdf = "<?xml version='1.0'?><robot name='" + config_data_->urdf_model_->getName() +
                                 "'></"
                                 "robot>";

  // Load a blank SRDF file to the parameter server
  if (!setSRDFFile(blank_srdf))
  {
    QMessageBox::warning(this, "Error Loading Files", "Failure loading blank SRDF file.");
    return false;
  }

  // Call a function that enables navigation
  Q_EMIT readyToProgress();

  // Progress Indicator
  progress_bar_->setValue(100);
  QApplication::processEvents();

  next_label_->show();  // only show once the files have been loaded

  ROS_INFO("Loading Setup Assistant Complete");
  return true;  // success!
```

## Update collisints

The collision table is generated [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/default_collisions_widget.cpp#L300) which runs in a thread.
This runs [this function ](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/tools/compute_default_collisions.cpp#L173)

```cpp
  unsigned int num_trials = density_slider_->value() * 1000 + 1000;  // scale to trials amount. Maximum density_slider_->value()=10'000
  double min_frac = (double)fraction_spinbox_->value() / 100.0; // "min collision for allway coliding" defaut = 0.95

  const bool verbose = true;  // Output benchmarking and statistics
  const bool include_never_colliding = true;
   auto link_pairs = moveit_setup_assistant::computeDefaultCollisions(
       config_data_->getPlanningScene(), collision_progress, include_never_colliding, num_trials, min_frac, verbose)
```

## Create virtual joint

Virtual joints are created [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/virtual_joints_widget.cpp#L425)

The joint of the models are loaded [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/virtual_joints_widget.cpp#L336)

[here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/virtual_joints_widget.cpp#L355C1-L355C1) we find if a virtual link is contained in the srdf by its name.

## Define Planning groups.

There are two widgets defined to do this job;

1. `PlanningGroupsWidget`
    - The "add group" button goes [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/planning_groups_widget.cpp#L809)
    - Then the add grouo screen is loaded [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/planning_groups_widget.cpp#L814C3-L814C18)
    - In the load grouop screen procedure, we push the group edig widget [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/planning_groups_widget.cpp#L623).
    - This is the screen to create the new group [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/planning_groups_widget.cpp#L1051)
    - The new group is created [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/planning_groups_widget.cpp#L1150)
    - The kinematics configuration of the group is set [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/planning_groups_widget.cpp#L1226)
    - The group trees are loaded [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/planning_groups_widget.cpp#L249)

2. `GroupEditWidget`

3. `KinematicChainWidget`
    - The links are loaded [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/kinematic_chain_widget.cpp#L141)
    - The base link is set [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/kinematic_chain_widget.cpp#L230)
    - The tip link is set [here](https://github.com/ros-planning/moveit/blob/1730d7b392a072bafaab37e6e3bb8ed11e58047e/moveit_setup_assistant/src/widgets/kinematic_chain_widget.cpp#L242)



# Create a config data from scratch

```cpp
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#include <moveit/rdf_loader/rdf_loader.h> // rdf_loader ns

int main(){
moveit_setup_assistant::MoveItConfigData config_data_;
rdf_loader::RDFLoader::loadXmlFileToString(config_data_->urdf_string_, urdf_file_path, { xacro_args });
config_data_->urdf_model_->initString(config_data_->urdf_string_);
config_data_->urdf_from_xacro_ = rdf_loader::RDFLoader::isXacroFile(urdf_file_path);

const std::string blank_srdf = "<?xml version='1.0'?><robot name='" + config_data_->urdf_model_->getName() +
                                 "'></"
                                 "robot>";
if (!config_data_->srdf_->initString(*config_data_->urdf_model_, blank_srdf))
{
    std::cout << "Error Loading Files", "SRDF file not a valid semantic robot description model.\n";
    return false;
}

// Now we can call MoveItConfigData::getRobotModel() and MoveItConfigData::getPlanningScene()
    unsigned int collision_progress:
   auto link_pairs = moveit_setup_assistant::computeDefaultCollisions(
       config_data_->getPlanningScene(), collision_progress, true, 10000, 0.95, true)

    wip_srdf_ = std::make_shared<srdf::SRDFWriter>(*config_data_->srdf_);


    wip_srdf_->disabled_collision_pairs_.clear();

  // Create temp disabled collision
  srdf::Model::CollisionPair dc;

  // copy the data in this class's LinkPairMap datastructure to srdf::Model::CollisionPair format
  for (const auto& item : link_pairs)
  {
    // Only copy those that are actually disabled
    if (item.second.disable_check)
    {
      dc.link1_ = item.first.first;
      dc.link2_ = item.first.second;
      dc.reason_ = moveit_setup_assistant::disabledReasonToString(item.second.reason);
      wip_srdf_->disabled_collision_pairs_.push_back(dc);
    }
  }
}
config_data_->loadAllowedCollisionMatrix(*wip_srdf_);

config_data_->changes |= MoveItConfigData::COLLISIONS;

// --------------- Set virtual joint

std::string vjoint_name = "foo";
// get all the links
    std::vector<std::string> link_names;

    for(const auto* link : config_data_->getRobotModel()->getLinkModels())
    {
            link_names.push_back(link->getName().c_str());
    }


  for (const auto& virtual_joint : config_data_->srdf_->virtual_joints_)
  {
    if (virtual_joint.name_.compare(vjoint_name) == 0)  // the names are the same
    {
        QMessageBox::warning(this, "Error Saving", "A virtual joint already exists with that name!");
        return;
    }
  }

  srdf::Model::VirtualJoint vj;

  vj->name_ = vjoint_name;
  vj->parent_frame_ = parent_name;
  vj->child_link_ = child_link_field_->currentText().toStdString();
  vj->type_ = joint_type_field_->currentText().toStdString();

 if (vj->child_link_ == config_data_->getRobotModel()->getRootLinkName())
      emit_frame_notice = true;
  config_data_->srdf_->virtual_joints_.push_back(vj);
  config_data_->updateRobotModel();

  config_data_->changes |= MoveItConfigData::VIRTUAL_JOINTS;


  /// create a new group

    srdf::Model::Group new_group;
    new_group.name_ = group_name;
    config_data_->srdf_->groups_.push_back(new_group);
    adding_new_group_ = true;  // remember this group is new
    config_data_->group_meta_data_[group_name].kinematics_solver_ = kinematics_solver;
    config_data_->group_meta_data_[group_name].kinematics_solver_search_resolution_ = kinematics_resolution_double;
    config_data_->group_meta_data_[group_name].kinematics_solver_timeout_ = kinematics_timeout_double;
    config_data_->group_meta_data_[group_name].goal_joint_tolerance_ = goal_joint_tolerance;
    config_data_->group_meta_data_[group_name].goal_position_tolerance_ = goal_position_tolerance;
    config_data_->group_meta_data_[group_name].goal_orientation_tolerance_ = goal_orientation_tolerance;
    config_data_->group_meta_data_[group_name].kinematics_parameters_file_ = kinematics_parameters_file;
    config_data_->group_meta_data_[group_name].default_planner_ = (default_planner == "None" ? "" : default_planner);config_data_->changes |= MoveItConfigData::GROUPS;
}

```
