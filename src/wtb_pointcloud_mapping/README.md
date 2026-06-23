# wtb_pointcloud_mapping

`wtb_pointcloud_mapping` implements the WTBInspector-style point cloud mapping and management layer for a Gazebo/Livox/PX4 workflow. It does pose-based direct stitching from ground-truth odometry, maintains a downsampled global point cloud, maintains a sparse 3D log-odds occupancy grid, and exposes occupied-voxel query interfaces for later tower, rotor plane, BRI ring, blade control-point, and blade-tip search modules.

The launch file starts `initial_pose_frame_node` by default. It captures the
first position from `/iris_0/mavros/vision_odom/odom`, defines that position as
the origin of the inspection `map` frame, publishes rebased odometry on
`/wtb/odom`, and broadcasts `world -> map -> iris_0/base_link`. Only translation
is rebased; the `map` axes remain aligned with Gazebo `world`.
The wind-generator model intentionally uses its own TF root
`wind_generator_world`, so the UAV/global `world` frame remains the only frame
named `world`.

This package is not LiDAR-SLAM. It does not implement ICP, NDT, LOAM, FAST-LIO, loop closure, or pose correction. It assumes `/iris_0/mavros/vision_odom/odom` provides `T_map_body`.

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

The launch file also starts the C++ `lidar_filter` node by default. The filter subscribes to `/livox/lidar`, preserves the original Livox frame and point coordinates, and publishes `/livox/lidar_filtered`. The current `mapping.yaml` subscribes to `/livox/lidar`; set `input/cloud_topic` to `/livox/lidar_filtered` if you want the mapper to consume the filtered stream. To skip starting the filter node:

```bash
roslaunch wtb_pointcloud_mapping wtb_mapping.launch use_lidar_filter:=false
```

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
/wtb/map_debug_info       std_msgs/String
/wtb/tower_candidate_cloud   sensor_msgs/PointCloud2
/wtb/tower_slice_centers    sensor_msgs/PointCloud2
/wtb/tower_control_points   sensor_msgs/PointCloud2
/wtb/tower_axis_marker      visualization_msgs/Marker
/wtb/tower_global_axis_marker visualization_msgs/Marker
/wtb/tower_ransac_line_marker visualization_msgs/Marker
/wtb/tower_debug_info       std_msgs/String
```

All output point clouds use `header.frame_id = world_frame`, which defaults to `map`.

## RViz

The launch file opens RViz automatically. If you open RViz manually, use:

```text
Fixed Frame: map
```

Add `PointCloud2` displays:

```text
/wtb/current_cloud_world
/wtb/global_cloud
/wtb/occupied_cloud
/wtb/tower_candidate_cloud
/wtb/tower_slice_centers
/wtb/tower_control_points
```

Enable `/wtb/free_cloud` only when needed, because free voxels can be much denser than occupied voxels.

Add `Marker` displays:

```text
/wtb/tower_axis_marker
/wtb/tower_global_axis_marker
/wtb/tower_ransac_line_marker
```

## Tower Axis Estimation

The mapper includes a WTBInspector-style tower center-axis estimator. The default method (`slice_center_axis`) is:

```text
Current frame map-frame point cloud
-> ROI crop around UAV, get tower candidate cloud
-> Split by Z height into slices
-> Each slice projected to XY plane
-> Circle fit (least squares) or centroid fallback per slice
-> Outlier rejection on slice centers
-> Fit local tower axis from slice centers
-> Temporal smoothing + jump rejection
-> Publish stable tower center axis
```

The old RANSAC-line-on-surface approach is preserved only as debug mode (`ransac_line_debug`). It fits `SACMODEL_PARALLEL_LINE` constrained to near-vertical and publishes a **yellow** line marker for comparison, but it is NOT used as the tower center axis.

Start it with the normal launch command:

```bash
roslaunch wtb_pointcloud_mapping wtb_mapping.launch
```

RViz expected behavior:

```text
Fixed Frame: map

PointCloud2:
  /wtb/tower_candidate_cloud    -- ROI-filtered tower points (yellow)
  /wtb/tower_slice_centers      -- Estimated slice center points (intensity = fit quality)
  /wtb/tower_control_points     -- Accumulated control points along height

Marker:
  /wtb/tower_axis_marker        -- GREEN: current smoothed tower center axis
  /wtb/tower_global_axis_marker -- RED: global axis fitted from control points
  /wtb/tower_ransac_line_marker -- YELLOW: RANSAC surface line (debug only, NOT the center axis)
```

Marker meanings:
- **Green** (`/wtb/tower_axis_marker`): Current smoothed tower center axis, estimated from slice centers with temporal filtering.
- **Red** (`/wtb/tower_global_axis_marker`): Global center axis fitted from all accumulated control points. Stabilizes as more control points accumulate.
- **Yellow** (`/wtb/tower_ransac_line_marker`): RANSAC-fitted surface vertical line. This may still cling to the tower surface. It is debug-only and does NOT represent the center axis.

Useful debug topic:

```bash
rostopic echo /wtb/tower_debug_info
```

The debug string reports method, candidate count, raw slices, valid slice centers, circle fit success, centroid fallback count, local/smoothed/global axis validity, jump rejection status, and control point count.

Common tuning:

```text
If candidate cloud contains too much ground: increase tower_axis/min_z.
If candidate cloud contains too many blade or background points: reduce tower_axis/roi_radius_xy or tower_axis/max_range_from_drone.
If slice centers are noisy: increase tower_axis/slice_height, adjust tower_axis/circle_fit_residual_threshold.
If circle fit fails often: increase tower_axis/tower_radius_max, ensure tower_axis/fallback_to_slice_centroid is true.
If axis still swings: reduce tower_axis/axis_point_alpha and tower_axis/axis_dir_alpha for stronger smoothing.
If axis jumps to wrong structure: reduce tower_axis/max_axis_lateral_jump, tower_axis/max_axis_angle_jump_deg.
If control points jump: reduce tower_axis/max_control_point_jump.
If the global axis is obviously offset: check LiDAR extrinsic, odom pose, map frame semantics, and whether the stitched cloud is duplicated.
```

## Parameters

The default config is `config/mapping.yaml`.

Important parameters:

```text
input/cloud_topic
input/odom_topic
input/world_frame
input/body_frame
input/lidar_frame
lidar_filter/input_topic
lidar_filter/output_topic
lidar_filter/ground_remove_z
lidar_filter/z_max
lidar_filter/range_min
lidar_filter/range_max
lidar_filter/front_angle_deg
lidar_filter/x_min
lidar_filter/y_abs
lidar_filter/use_3d_range
sync/max_time_diff
sync/strict_time_sync
extrinsic/lidar_to_body_xyz
extrinsic/lidar_to_body_rpy
filter/min_range
filter/max_range
filter/z_min
filter/z_max
global_cloud/voxel_leaf_size
global_cloud/max_points
occupancy_grid/resolution
occupancy_grid/max_ray_length
tower_axis/enable
tower_axis/input_cloud_mode
tower_axis/voxel_leaf_size
tower_axis/min_z
tower_axis/max_z
tower_axis/roi_radius_xy
tower_axis/min_range_from_drone
tower_axis/max_range_from_drone
tower_axis/ransac_distance_threshold
tower_axis/ransac_max_iterations
tower_axis/min_inliers
tower_axis/vertical_angle_threshold_deg
tower_axis/min_control_point_spacing
tower_axis/max_control_point_jump
tower_axis/require_height_increase
tower_axis/fit_global_axis_min_points
debug/publish_free_cloud
```

`extrinsic/lidar_to_body_xyz` and `extrinsic/lidar_to_body_rpy` define `T_body_lidar`, so they transform a point from LiDAR coordinates into body coordinates. RPY is in radians and uses yaw-pitch-roll composition:

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

If `input/use_tf` is `true`, the node looks up `T_body_lidar` from TF using `lookupTransform(body_frame, lidar_frame, stamp)`. The default is `false`, so the YAML extrinsic is used.

`lidar_filter/*` controls the C++ raw Livox filter node. Its filtering logic matches `scripts/filter_livox_forward.py`: optional ground removal, `z_max`, `x_min`, 2D or 3D range, front angle, and optional `y_abs`. It does not transform points; the output stays in the original `/livox/lidar` coordinate frame.

## Debugging

Start with:

```bash
rostopic echo /wtb/map_debug_info
```

The debug string includes current LiDAR points, filtered world points, global cloud points, total occupancy voxels, occupied voxels, cloud-odom time difference, and processing time.

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
