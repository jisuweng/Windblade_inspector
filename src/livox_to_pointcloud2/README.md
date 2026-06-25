# livox_to_pointcloud2

ROS1 node that subscribes to `livox_laser_simulation/CustomMsg`, applies the
Livox FOV/range crop, and publishes the cropped result in two formats:

- `/livox/lidar_filtered`: cropped cloud as `livox_laser_simulation/CustomMsg`.
- `/livox/debug/lidar_filtered_pcl2`: cropped cloud as `sensor_msgs/PointCloud2` for RViz/debug.

The default crop tests points in a body-aligned filter frame using the bundled
`iris_mid360` LiDAR mount pose (`x=0.07`, `z=0.072`, `pitch=0.3925`). This
keeps the crop region aligned with the UAV body instead of the tilted LiDAR.
Point coordinates are not transformed before publishing, so downstream mapping
can still apply its normal `T_body_lidar` extrinsic exactly once. The input
`header.frame_id` is preserved unless `output_frame_id` is set explicitly.

## Build and run

```bash
cd ~/WTBinspector_ws
catkin build livox_to_pointcloud2
source devel/setup.bash
roslaunch livox_to_pointcloud2 livox_to_pointcloud2.launch
```

The debug PointCloud2 contains the same cropped points as `/livox/lidar_filtered`
with fields `x`, `y`, `z` (`FLOAT32`), `t` (`UINT32`), `intensity` (`FLOAT32`),
`tag` (`UINT8`), and `line` (`UINT8`). The Livox output preserves
`CustomPoint` fields and message metadata.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `input_topic` | `/livox/lidar` | `livox_laser_simulation/CustomMsg` input |
| `cropped_topic` | `/livox/lidar_filtered` | Cropped Livox CustomMsg |
| `debug_pcl2_topic` | `/livox/debug/lidar_filtered_pcl2` | Cropped PointCloud2 for RViz/debug |
| `publish_debug_pcl2` | `true` | Publish `debug_pcl2_topic` |
| `horizontal_fov_deg` | `80.0` | Total horizontal FOV, i.e. +/-40 degrees |
| `vertical_fov_deg` | `80.0` | Total vertical FOV, i.e. +/-40 degrees |
| `use_vertical_fov` | `true` | Apply `vertical_fov_deg`; disable for only horizontal sector + z/range crop |
| `min_range` | `0.0` | Minimum range in metres |
| `max_range` | `70.0` | Maximum 3D range in metres |
| `use_3d_range` | `false` | Use 3D range instead of horizontal XY range |
| `z_min`, `z_max` | very wide | Height limits in the filter frame |
| `x_min` | very negative | Minimum forward coordinate in the filter frame |
| `y_abs` | `-1.0` | Optional lateral half-width; non-positive disables it |
| `filter_frame_*` | `0.07, 0, 0.072, 0, 0.3925, 0` | `T_filter_lidar` used only for crop decisions |
| `output_frame_id` | empty | Empty preserves the input frame |
| `queue_size` | `10` | Subscriber and publisher queue depth |

## RViz

RViz can display `/livox/debug/lidar_filtered_pcl2` directly. The mapper consumes
`/livox/lidar_filtered`, which stays in Livox CustomMsg format.
