# wtb_pointcloud_mapping

`wtb_pointcloud_mapping` implements the WTBInspector-style point cloud mapping layer for a Gazebo/Livox/PX4 workflow. It does pose-based direct stitching from ground-truth odometry, maintains a downsampled global point cloud, and maintains a sparse 3D log-odds occupancy grid.

The launch file starts `initial_pose_frame_node` by default. It captures the
first position from `/iris_0/mavros/vision_odom/odom`, defines that position as
the origin of the inspection `map` frame, publishes rebased odometry on
`/wtb/odom`, and broadcasts `world -> map -> iris_0/base_link`. Only translation
is rebased; the `map` axes remain aligned with Gazebo `world`.
The wind-generator model intentionally uses its own TF root
`wind_generator_world`, so the UAV/global `world` frame remains the only frame
named `world`.

This package is not LiDAR-SLAM. It does not implement ICP, NDT, LOAM, FAST-LIO, loop closure, or pose correction. It assumes `/iris_0/mavros/vision_odom/odom` provides `T_map_body`.

## Structure

The mapping package is split into a small ROS adapter and reusable mapping
modules:

```text
include/wtb_pointcloud_mapping/mapping_config.h
src/mapping_config.cpp
  Loads YAML/ROS parameters into typed config structs.

include/wtb_pointcloud_mapping/ros_conversions.h
src/ros_conversions.cpp
  Converts Livox CustomMsg, Odometry, Eigen transforms, and PointCloud2.

include/wtb_pointcloud_mapping/mapping_pipeline.h
src/mapping_pipeline.cpp
  Owns the mapping flow: lidar cloud -> map cloud -> global cloud -> occupancy grid.

include/wtb_pointcloud_mapping/pointcloud_stitcher.h
src/pointcloud_stitcher.cpp
  Handles map-frame transformation and global cloud accumulation.

include/wtb_pointcloud_mapping/occupancy_grid_3d.h
src/occupancy_grid_3d.cpp
  Sparse 3D log-odds voxel grid and ray-casting updates.

include/wtb_pointcloud_mapping/wtb_mapping_node.h
src/wtb_mapping_node.cpp
  ROS subscriptions, TF lookup, publishers, and debug output only.
```

`CMakeLists.txt` builds the reusable `wtb_mapping_core` library first, then links
the `wtb_mapping_node` executable against it.

## Build

```bash
cd ~/catkin_ws/src
# Put wtb_pointcloud_mapping here.
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

In this checkout the package is already under `WTBinspector_ws/src`, so the local build command is:

```bash
cd /home/byz/WTBinspector_ws
catkin build wtb_pointcloud_mapping
source devel/setup.bash
```

This workspace vendors a `livox_laser_simulation` package that defines `CustomMsg.msg` and `CustomPoint.msg`, so `source devel/setup.bash` from `WTBinspector_ws` is enough.

## Launch

```bash
roslaunch wtb_pointcloud_mapping wtb_mapping.launch
```

The launch file starts RViz by default with `rviz/wtb_mapping.rviz`. To run the mapper without RViz:

```bash
roslaunch wtb_pointcloud_mapping wtb_mapping.launch use_rviz:=false
```

The mapper has a single point-cloud subscription:
`/livox/lidar_filtered`. That topic is produced by `livox_to_pointcloud2` after
FOV/range cropping. The mapping launch file does not start any LiDAR filter node
and the mapping pipeline does not apply range/FOV/Z filtering to the input cloud.

## Inputs

Check the input topics:

```bash
rostopic info /livox/lidar
rostopic info /livox/lidar_filtered
rostopic info /iris_0/mavros/vision_odom/odom
```

Check the Livox simulation message fields:

```bash
rosmsg show livox_laser_simulation/CustomMsg
rosmsg show livox_laser_simulation/CustomPoint
```

The expected `CustomPoint` fields are `x`, `y`, `z`, and `reflectivity`. If your message uses a different intensity field, update `convertLivoxCustomMsgToPCL()` in `src/wtb_mapping_node.cpp`.

Check the odometry frame semantics:

```bash
rostopic echo -n 1 /iris_0/mavros/vision_odom/odom/header
rostopic echo -n 1 /iris_0/mavros/vision_odom/odom/child_frame_id
```

The implementation treats `odom.pose.pose` as the UAV body pose in `map`, i.e. `T_map_body`.

## Outputs

The node publishes:

```text
/wtb/current_cloud_world  sensor_msgs/PointCloud2
/wtb/global_cloud         sensor_msgs/PointCloud2
/wtb/occupied_cloud       sensor_msgs/PointCloud2
/wtb/free_cloud           sensor_msgs/PointCloud2, enabled by debug/publish_free_cloud
/wtb/uav_path             nav_msgs/Path
/wtb/map_debug_info       std_msgs/String
```

All output point clouds use `header.frame_id = world_frame`, which defaults to `map`.

`wtb_mapping.launch` also starts `wtb_tower_detector_node` by default. It consumes
`/wtb/current_cloud_world` and `/wtb/odom`; the point-cloud stitching and occupancy
pipeline remain unchanged. To disable it:

```bash
roslaunch wtb_pointcloud_mapping wtb_mapping.launch use_tower_detector:=false
```

The tower detector publishes:

```text
/wtb/tower_detector/status          std_msgs/String
/wtb/tower_detector/control_point   geometry_msgs/PointStamped
/wtb/tower_detector/control_pose    geometry_msgs/PoseStamped
/wtb/tower_detector/axis_direction  geometry_msgs/Vector3Stamped
/wtb/tower_detector/markers         visualization_msgs/MarkerArray
/wtb/tower_detector/inlier_cloud    sensor_msgs/PointCloud2
/wtb/tower_detector/path            nav_msgs/Path
/wtb/tower_detector/diameter        std_msgs/Float64
/wtb/tower_detector/radius          std_msgs/Float64
```

It also mirrors the geometry into ROS params under `/wtb/tower_detector/*` and,
by default, `/tower/*` for compatibility with older `tower_detector.py` consumers.

## RViz

The launch file opens RViz automatically with `rviz/wtb_mapping.rviz`. If you open
RViz manually, use:

```text
Fixed Frame: map
```

Add `PointCloud2` displays:

```text
/wtb/current_cloud_world
/wtb/global_cloud
/wtb/occupied_cloud
/wtb/uav_path
```

Enable `/wtb/free_cloud` only when needed, because free voxels can be much denser than occupied voxels.

## Parameters

The default config is `config/mapping.yaml`.

Important parameters:

```text
input/cloud_topic
input/odom_topic
input/world_frame
input/body_frame
input/lidar_frame
initial_pose_frame/input_odom_topic
initial_pose_frame/output_odom_topic
sync/max_time_diff
sync/strict_time_sync
extrinsic/lidar_to_body_xyz
extrinsic/lidar_to_body_rpy
global_cloud/voxel_leaf_size
global_cloud/max_points
occupancy_grid/resolution
occupancy_grid/max_ray_length
path/topic
path/min_distance
path/max_points
debug/publish_free_cloud
```

`extrinsic/lidar_to_body_xyz` and `extrinsic/lidar_to_body_rpy` define `T_body_lidar`, so they transform a point from LiDAR coordinates into body coordinates. RPY is in radians and uses yaw-pitch-roll composition:

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

If `input/use_tf` is `true`, the node looks up `T_body_lidar` from TF using `lookupTransform(body_frame, lidar_frame, stamp)`. The default is `false`, so the YAML extrinsic is used.

## Debugging

Start with:

```bash
rostopic echo /wtb/map_debug_info
```

The debug string includes current LiDAR points, current world points, global cloud points, total occupancy voxels, occupied voxels, cloud-odom time difference, and processing time.

If the stitched cloud is doubled, thick, stretched, or tilted, check these first:

```text
LiDAR-to-body extrinsic direction and units
odom pose really means T_map_body
LiDAR point coordinates are in the expected lidar_frame
cloud and odom timestamps
RPY values are radians
world_frame is map in RViz
```

The first quick validation can use zero extrinsic. If the LiDAR is visibly offset or rotated relative to the UAV body, update `lidar_to_body_xyz` and `lidar_to_body_rpy`.

For the bundled `iris_mid360` simulation, Mid360 is mounted forward with a 22.5 degree pitch. The default `mapping.yaml` therefore uses the SDF pose `x=0.07 m`, `z=0.072 m`, `pitch=0.3925 rad` for `T_body_lidar`.
