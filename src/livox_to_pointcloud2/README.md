# livox_to_pointcloud2

ROS1 node that subscribes to `livox_laser_simulation/CustomMsg` and publishes
one converted PointCloud2 stream and one cropped Livox stream:

- `/livox/points_raw`: complete cloud as `sensor_msgs/PointCloud2`.
- `/livox/points`: cropped cloud as `livox_laser_simulation/CustomMsg`.

The default 60 degree horizontal and 60 degree vertical field of view keeps
points within +/-30 degrees of the Livox +X forward axis. Point coordinates are
not transformed. The input `header.frame_id` is preserved unless
`output_frame_id` is set explicitly.

## Build and run

```bash
cd ~/WTBinspector_ws
catkin build livox_to_pointcloud2
source devel/setup.bash
roslaunch livox_to_pointcloud2 livox_to_pointcloud2.launch
```

The converted PointCloud2 contains `x`, `y`, `z` (`FLOAT32`), `t` (`UINT32`),
`intensity` (`FLOAT32`), `tag` (`UINT8`), and `line` (`UINT8`). The cropped
output preserves Livox `CustomPoint` fields and message metadata.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `input_topic` | `/livox/lidar` | `livox_laser_simulation/CustomMsg` input |
| `converted_topic` | `/livox/points_raw` | Complete converted PointCloud2 |
| `cropped_topic` | `/livox/points` | Cropped Livox CustomMsg |
| `horizontal_fov_deg` | `60.0` | Total horizontal FOV |
| `vertical_fov_deg` | `60.0` | Total vertical FOV |
| `min_range` | `0.0` | Minimum 3D range in metres |
| `max_range` | `-1.0` | Maximum 3D range; non-positive disables it |
| `output_frame_id` | empty | Empty preserves the input frame |
| `queue_size` | `10` | Subscriber and publisher queue depth |

## RViz

RViz can display `/livox/points_raw` directly. The cropped `/livox/points` is a
Livox CustomMsg intended for Livox-aware downstream nodes and cannot be added as
an RViz PointCloud2 display without converting it again.
