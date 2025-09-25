![bonxai_ros_node_img](doc/bonxai_ros.drawio.png)

# 1. Installation

1. Navigate towards your workspace `src` folder
```bash
cd ~/ws_bonxai/src
```
2. Git clone the Bonxai package
```bash
git clone https://github.com/facontidavide/Bonxai.git
```
3. Go back to your `ws_bonxai` and build
```bash
cd ~/ws_bonxai && colcon build --packages-select bonxai_ros --symlink-install
```
4. Source your workspace setub.bash file

```bash
source ~/ws_bonxai/install/setup.bash
```

# 2. Basic Usage

The Bonxai Server Node is launched with the following command, for basic deployment with a minimal number of tunnable parameters:

```bash
ros2 launch bonxai_ros bonxai_mapping.launch.xml
```

There is another launch file that allows you to modify all the available parameters within the params/bonxai_params.yaml and launch the bonxai_server_node along RViz2.

```bash
ros2 launch bonxai_ros bonxai_mapping.launch.py
```

<p align="center">
  <img src="doc/nice_vis.gif" alt="Sublime's custom gif"/>
</p>

# 3. ROS 2 node API

<p align="center">
  <img src="doc/bonxai_ros_node.drawio.png" alt="Sublime's custom image"/>
</p>

## 3.1 Bonxai Server

Bonxai_server builds and distributes volumetric 3D occupancy maps for obstacle avoidance, planning or visualization. The map is incrementally built from incoming range data (as PointCloud2) either from RGBD cameras or LiDARs.

## 3.1.1 Subscribed Topics

 `cloud_in (sensor_msgs/msg/PointCloud2)`

Incoming 3D point cloud for scan integration. You need to remap this topic to your sensor data and provide a tf transform between the sensor data and the static map frame. The frame_id of the point cloud has to be the sensor frame. Raytracing (clearing of free space) is always done from the origin of this frame at the time of the point cloud.


## 3.1.2 Published Topics

`bonxai_point_cloud_centers (sensor_msgs/msg/PointCloud2)`

The centers of all occupied voxels as point cloud, useful for visualization. For an accurate visualization, manually change the pointcloud size in Rviz2 to the respective voxel resolution.

## 3.1.3 Services

`~reset (std_srvs/srv/Empty)`

Resets the complete map.

`~pause_mapping (std_srvs/srv/SetBool)`

Pauses the complete mapping. "True" pauses the mapping, "False" resumes it.

## 3.1.4 Parameters

`~frame_id (string, default: /map)`

Static global frame in which the map will be published. A transform from sensor data to this frame needs to be available when dynamically building maps.

`~resolution (float, default: 0.05)`

Resolution in meter (voxel_size) for the map when starting with an empty map.

`~base_frame_id (string, default: base_footprint)`

The robot's base frame.

`~sensor_model/max_range (float, default: -1 (unlimited))`

Maximum range in meter for inserting point cloud data when dynamically building a map. Limiting the range to something useful (e.g. 5m) prevents spurious erroneous points far away from the robot.

`~sensor_model/[hit|miss] (float, default: 0.7 / 0.4)`

Probabilities for hits and misses in the sensor model when dynamically building a map

`~sensor_model/[min|max] (float, default: 0.12 / 0.97)`

Minimum and maximum probability for clamping when dynamically building a map

`~latch (bool, default: True for a static map, false if no initial map is given)`

Whether topics are published latched or only once per change. For maximum performance when building a map (with frequent updates), set to false. When set to true, on every map change all topics and visualizations will be created.

`~point_cloud_[min|max]_z (float, default: -/+ infinity)`

Minimum and maximum height of points to consider for insertion in the callback. Any point outside of this intervall will be discarded before running any insertion.

`~occupancy_[min|max]_z (float, default: -/+ infinity)`

Minimum and maximum height of occupied cells to be consider in the final map. This ignores all occupied voxels outside of the interval when sending out visualizations and collision maps, but will not affect the actual Bonxai map representation.

`~pause_mapping (bool, default: False for mapping, True for pausing)`

Pauses the complete mapping. "True" pauses the mapping, "False" resumes it.

    ros2 service call /bonxai_server_node/pause_mapping std_srvs/srv/SetBool "{data: true}"

### 3.1.4.1 Temporal & Spatial Filtering Parameters

To mitigate transient LiDAR artifacts (rain, dust, multi-path “ghost” points) an advanced temporal
and spatial confirmation system is provided. These replace (but remain backward compatible with)
the original simple `filter.confirm_hits` / `filter.confirm_window` logic.

`~filter.confirm_hits (int, default: 2)`
: Legacy alias for minimum observations required. If set > 1 but advanced params are left at
  defaults, it maps onto `filter.required_observations`.

`~filter.confirm_window (int, default: 30)`
: Legacy timeout (frames) between first and last observation for staged voxel. Still enforced in
  addition to advanced timeouts when > 0.

`~filter.window_frames (int, default: 32, range: 1..64)`
: Size (in map update frames) of the sliding temporal window encoded as a bit-mask per staged voxel.

`~filter.required_observations (int, default: 3)`
: Number of set bits (unique frames with observations inside `window_frames`) required for
  promotion to occupied.

`~filter.min_neighbor_support (int, default: 0)`
: Require at least this many 6-connected neighbours already occupied OR also staged (observed) at
  promotion time. Use 1–2 to suppress isolated floating points; 0 disables spatial gating.

`~filter.stale_frames (int, default: 64)`
: If a staged voxel hasn’t been re-observed within this many frames it is discarded.

`~filter.deoccupy_frames (int, default: 0 (disabled))`
: If > 0, an occupied voxel not reconfirmed within this many frames is gradually decayed toward
  free (allows clearing dynamic clutter / moving objects).

`~filter.fractional_hits (bool, default: true)`
: If true, partial log-odds are accumulated while staged so that promotion adds the aggregated value.
  If false, a single full hit log-odds increment is applied on promotion.

#### Example YAML excerpt
```yaml
bonxai_server_node:
  ros__parameters:
    filter.confirm_hits: 2            # backward compatible
    filter.confirm_window: 30
    filter.window_frames: 32
    filter.required_observations: 3
    filter.min_neighbor_support: 1
    filter.stale_frames: 64
    filter.deoccupy_frames: 0
    filter.fractional_hits: true
```

#### Tuning Guidelines
* Increase `required_observations` (3→4) for harsher rain filtering.
* Add `min_neighbor_support: 1` to remove isolated specs while retaining contiguous structures.
* Reduce `window_frames` if you want faster confirmation (e.g., fast spinning sensor) but balance
  with potential false positives.
* Enable `deoccupy_frames` to allow clearing transient obstacles (e.g., moving vehicles) without
  global reset.
* Disable `fractional_hits` if you prefer a hard threshold promotion behavior.


## 3.1.5 Required tf Transforms

`sensor data frame → /map (static world frame, changeable with parameter frame_id)`

Required transform of sensor data into the global map frame if you do scan integration. This information needs to be available from an external SLAM or localization node.
