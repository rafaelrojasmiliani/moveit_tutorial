

create the package
`catkin create pkg d_octomap_monitor --catkin-deps roscpp moveit_core moveit_ros_planning_interface --system-deps Eigen3`


- The planning scene monitor declares the octopam [here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h#L514)


## `OccupancyMapMonitor`

[Declared here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/include/moveit/occupancy_map_monitor/occupancy_map_monitor.h#L56) and [defined here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp#L48).

In its initialization it looks for the sensors [here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp#L106).

`sensors` is a struct with a member "sensor_plugin".
For each sensor, a occupancy map updater is instantiated and initizalised.

```C++
  if (nh_.getParam("sensors", sensor_list))
  {
    if (sensor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      for (int32_t i = 0; i < sensor_list.size(); ++i)
      {

        std::string sensor_plugin = std::string(sensor_list[i]["sensor_plugin"]);
        if (sensor_plugin.empty() || sensor_plugin[0] == '~')
        {
          ROS_INFO_STREAM_NAMED(LOGNAME, "Skipping octomap updater plugin '" << sensor_plugin << "'");
        }

        if (!updater_plugin_loader_)
        {
            updater_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<OccupancyMapUpdater>>(
                "moveit_ros_perception", "occupancy_map_monitor::OccupancyMapUpdater");
        }

        OccupancyMapUpdaterPtr up;
        up = updater_plugin_loader_->createUniqueInstance(sensor_plugin);
        up->setMonitor(this);
        up->setParams(sensor_list[i])
        up->initialize()

        // addUpdater(up);  Begin -------------------------
		  auto &updater = up;
        map_updaters_.push_back(updater);
        updater->publishDebugInformation(debug_info_);
        if (map_updaters_.size() > 1)
        {
          mesh_handles_.resize(map_updaters_.size());
          // when we had one updater only, we passed the transform cache callback directly to that updater
          if (map_updaters_.size() == 2)
          {
            map_updaters_[0]->setTransformCacheCallback(
                [this](const std::string& frame, const ros::Time& stamp, ShapeTransformCache& cache) {
                  return getShapeTransformCache(0, frame, stamp, cache);
                });
            map_updaters_[1]->setTransformCacheCallback(
                [this](const std::string& frame, const ros::Time& stamp, ShapeTransformCache& cache) {
                  return getShapeTransformCache(1, frame, stamp, cache);
                });
          }
          else
            map_updaters_.back()->setTransformCacheCallback(
                [this, i = map_updaters_.size() - 1](const std::string& frame, const ros::Time& stamp,
                                                     ShapeTransformCache& cache) {
                  return getShapeTransformCache(i, frame, stamp, cache);
                });
        }
        else
          updater->setTransformCacheCallback(transform_cache_callback_);
        // addUpdater(up);  END ------------------------
      }
    else
      ROS_ERROR_NAMED(LOGNAME, "List of sensors must be an array!");
  }
  else
    ROS_INFO_NAMED(LOGNAME, "No 3D sensor plugin(s) defined for octomap updates");
```

### ROS API

| Service Name | Method |
| -----------  | -----  |
| `"save_map"` | [`OccupayMapMonitor::saveMapCallback`](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp#L314) |
| `"load_map"` | [`OccupayMapMonitor::loadMapCallback`](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp#L331) |

## `OccupancyMapUpdater`


[Declared here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/include/moveit/occupancy_map_monitor/occupancy_map_updater.h#L58) and [defined here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/src/occupancy_map_updater.cpp#L44)


| Virtual Function | arguents | function |
| ---------------- | -------  | -------- |
| `bool setParams(XmlRpc::XmlRpcValue& params)` | | |
| `bool initialize()` |||
| `void start()` |||
| `void stop()` |||
| `ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape)` |||
| `void forgetShape(ShapeHandle handle)` |||


| Protected Variables | arguents | function |
| ---------------- | -------  | -------- |
| `OccupancyMapMonitor* monitor_` |||
| `std::string type_` |||
| `collision_detection::OccMapTreePtr tree_` |||
| `TransformCacheProvider transform_provider_callback_` |||
| `ShapeTransformCache transform_cache_` |||
| `bool debug_info_` |||


| Protected Function | |
| ------------------ | -- |
| [`bool updateTransformCache(const std::string& target_frame, const ros::Time& target_time)`](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/src/occupancy_map_updater.cpp#L73) | |
| [`static void readXmlParam(XmlRpc::XmlRpcValue& params, const std::string& param_name, double* value)`](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/src/occupancy_map_updater.cpp#L56) ||
| [`static void readXmlParam(XmlRpc::XmlRpcValue& params, const std::string& param_name, unsigned int* value)`](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/occupancy_map_monitor/src/occupancy_map_updater.cpp#L67) ||


## Implementations of `OccupancyMapUpdater`

### `DepthImageOctomapUpdater`

This publins implement components from the [`image_transport`](http://wiki.ros.org/image_transport) package.
`image_transport` provides classes and nodes for transporting images in arbitrary representations, that the developer only sees sensor_msgs/Image messages.

`DepthImageOctomapUpdater` is declared [here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_ros/perception/depth_image_octomap_updater/include/moveit/depth_image_octomap_updater/depth_image_octomap_updater.h) and [defined here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/perception/depth_image_octomap_updater/src/depth_image_octomap_updater.cpp#L53)


| Subscribed topics |  |
| ----------------  | --- |
| `image_topic_`, set with `setParams` | [callback](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/perception/depth_image_octomap_updater/src/depth_image_octomap_updater.cpp#L220) |

#### ROS API

Parameters

| PArameter name | type |  |
| -------------  | --- | --- |
|  "sensor_type"  | `string` | Not used|
| (optional, default value is "deep") "image_topic"   | `string` ||
| (optional) "queue_size"   | `int` ||
| `"near_clipping_plane_distance"`||
| `"far_clipping_plane_distance"`||
| `"shadow_threshold"`||
| `"padding_scale"`||
| `"padding_offset"`||
| (optional) "max_update_rate" | int |
| `"skip_vertical_pixels"`||
| `"skip_horizontal_pixels"`||
| (optional) `"filtered_cloud_topic"` |||
| (optional) `"ns"` | string| namespace to publish|


####### Example

```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/DepthImageOctomapUpdater
    image_topic: /camera/depth_registered/image_raw
    queue_size: 5
    near_clipping_plane_distance: 0.3
    far_clipping_plane_distance: 5.0
    shadow_threshold: 0.2
    padding_scale: 4.0
    padding_offset: 0.03
    max_update_rate: 1.0
    filtered_cloud_topic: filtered_cloud
    ns: kinect
```

### `PointCloudOctomapUpdater`

It uses the [`message_filters` library](http://wiki.ros.org/message_filters).
`message_filters` is a utility library to collect commonly used message "filtering" algorithms.
A message filter is defined as something which a message arrives into and may or may not be spit back out of at a later point in time.

An example is the time synchronizer, which takes in messages of different types from multiple sources, and outputs them only if it has received a message on each of those sources with the same timestamp.

In particular it implements [`tf2_ros::MessageFilter`](https://github.com/ros/geometry2/blob/2efd59d134d96f8e73f4a4bdb77a2c08a227057e/tf2_ros/include/tf2_ros/message_filter.h#L104), which implements a filter which only passes messages through once there is transform data available.


`PointCloudOctomapUpdater` is [declared here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/perception/pointcloud_octomap_updater/include/moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h#L51) and [defined here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp#L51).


#### ROS API

| PArameter name | type |  |
| -------------  | --- | --- |
| `"point_cloud_topic"` |||
| `"max_range"`|||
| `"padding_offset"`|||
| `"padding_scale"`|||
| `"point_subsample"`|||
| (optional) `"max_update_rate"`|||
| (optional) `"filtered_cloud_topic"`| string |actives [this code](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp#L243) and instantiates a publisher [here](https://github.com/ros-planning/moveit/blob/9cc7e8fb0d5b9ceb09d5ba68b524e7a4ab7ca02f/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp#L108)|
| (optional) `"ns"` | string| namespace to publish|


##### Example
```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth_registered/points
    max_range: 5.0
    point_subsample: 1
    padding_offset: 0.1
    padding_scale: 1.0
    max_update_rate: 1.0
    filtered_cloud_topic: filtered_cloud
    ns: kinect
```
