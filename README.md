![Bonxai](doc/bonxai.png)

Bonxai is a library that implements a compact hierarchical data structure that
can store and manipulate volumetric data, discretized on a three-dimensional
grid (AKA, a "Voxel Grid").

Bonxai data structure is:

- **Sparse**: it uses only a fraction of the memory that a dense 3D voxel grid would use.
- **Unbounded**: you don't need to define the boundary of the 3D space (*).

>(*) The dimension of the 3D space is virtually "infinite":
since **32-bits indices** are used, given a voxel size of **1 cm**,
the maximum range of the X, Y and Z coordinates would be about **40.000 Km**.
As a reference **the diameter of planet Earth is 12.000 Km**.

If you are familiar with [Octomap](https://octomap.github.io/) and Octrees, you know
that those data structures are also sparse and unbounded.

On the other hand, Bonxai is **much faster** and, in some cases, even more memory-efficient
than an Octree.

This work is strongly influenced by [OpenVDB](https://www.openvdb.org/) and it can be considered
an implementation of the original paper, with a couple of non-trivial changes:

    K. Museth,
    “VDB: High-Resolution Sparse Volumes with Dynamic Topology”,
    ACM Transactions on Graphics 32(3), 2013. Presented at SIGGRAPH 2013.

You can read the previous paper [here](http://www.museth.org/Ken/Publications_files/Museth_TOG13.pdf).

There is also some overlap with this other paper, but their implementation is much** simpler,
even if conceptually similar:

     Eurico Pedrosa, Artur Pereira, Nuno Lau
     "A Sparse-Dense Approach for Efficient Grid Mapping"
     2018 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)


**Bonxai** is currently under development and I am building this mostly for fun and for
educational purposes. Don't expect any API stability for the time being.

# Benchmark (preliminary)

Take these numbers with a grain of salt, since they are preliminary and the benchmark is
strongly influenced by the way the data is stored.
Anyway, they gave you a fair idea of what you may expect, in terms of performance.

```
-------------------------------------------
Benchmark                     Time
-------------------------------------------
Bonxai_Create              1165 us
Octomap_Create            25522 us

Bonxai_Update               851 us
Octomap_Update             3824 us

Bonxai_IterateAllCells      124 us
Octomap_IterateAllCells     698 us
```

- **Create** refers to creating a new VoxelGrid from scratch
- **Update** means modifying the value of an already allocated VoxelGrid.
- **IterateAllCells** will get the value and the coordinates of all the existing cells.

# How to use it

The core of **Bonxai** is a header-only library that you can simply copy into your project
and include like this:

```c++
#include "bonxai/bonxai.hpp"
```

To create a VoxelGrid, where each cell contains an integer value and has size 0.05.

```c++
double voxel_resolution = 0.05;
Bonxai::VoxelGrid<int> grid( voxel_resolution );
```

Nothing prevents you from having more complex cell values, for instance:

```c++
Bonxai::VoxelGrid<Eigen::Vector4d> vector_grid( voxel_resolution );
// or
struct Foo {
 int a;
 double b;
};
Bonxai::VoxelGrid<Foo> foo_grid( voxel_resolution );
```

To insert values into a cell with coordinates x, y and z, use a
`VoxelGrid::Accessor` object.
In the next code sample, we will create a dense cube of cells with value 42:

```c++
// Each cell will contain a `float` and it will have size 0.05
double voxel_resolution = 0.05;
Bonxai::VoxelGrid<float> grid( voxel_resolution );

// Create this accessor once, and reuse it as much as possible.
auto accessor = grid.createAccessor();

// Create cells with value 42.0 in a 1x1x1 cube.
// Given voxel_resolution = 0.05, this will be equivalent
// to 20x20x20 cells in the grid.

for( double x = 0; x < 1.0; x += voxel_resolution ) {
  for( double y = 0; y < 1.0; y += voxel_resolution ) {
    for( double z = 0; z < 1.0; z += voxel_resolution ) {
      // discretize the position {x,y,z}
      Bonxai::CoordT coord = grid.posToCoord(x, y, z);
      accessor.setValue( coord, 42.0 );
    }
  }
}

// You can read (or update) the value of a cell as shown below.
// If the cell doesn't exist, `value_ptr` will be `nullptr`,

Bonxai::CoordT coord = grid.posToCoord(x, y, z);
float* value_ptr = accessor.value( coord );
```

## Note about multi-threading

`Bonxai::VoxelGrid` is **not** thread-safe, for write operations.

If you want to access the grid in **read-only** mode, you can
use multi-threading, but each thread should have its own
`accessor`.

# Roadmap

- [x] serialization to/from file.
- [x] full implementation of the Octomap algorithm (ray casting + probability map).
- [x] integration with ROS 2.
- [ ] RViz2 visualization messages.
- [ ] integration with [FCL](https://github.com/flexible-collision-library/fcl) for collision detection (?)

# Frequently Asked Question

**What is the point of reimplementing OpenVDB?**

- The number one reason is to have fun and to learn something new :)
- I want this library to be small and easy to integrate into larger projects.
  The core data structure is less than 1000 lines of code.
- It is not an "exact" rewrite, I modified a few important aspects of the algorithm
    to make it slightly faster, at least for my specific use cases.

**How much memory does it use, compared with Octomap?**

It is... complicated.

If you need to store very sparse point clouds, you should expect Bonxai to use more memory (20-40% more).
If the point cloud is relatively dense, Bonxai might use less memory than Octomap (less than half).

## ROS 2 Node Parameters (bonxai_ros)

The ROS 2 mapping server node (`bonxai_server_node`) exposes a focused set of parameters (see `bonxai_ros/params/bonxai_params.yaml`) to control spatial resolution, occupancy update probabilities, and a lightweight temporal confirmation filter.

| Parameter | Type | Default | Meaning / Tuning Notes |
|-----------|------|---------|------------------------|
| `resolution` | double | 0.02 | Voxel edge length (m). Smaller = more detail, higher memory + CPU. Doubling typically cuts memory/time by ~8× (volume). |
| `frame_id` | string | `map` | Global fixed frame for published map. Must exist in TF. |
| `base_frame_id` | string | `base_footprint` | Robot body frame from which sensor data is transformed into `frame_id`. |
| `occupancy_min_z` | double | -100.0 | Minimum accepted Z (m) in `frame_id`. Tighten (e.g. -1.5) to prune irrelevant vertical regions. |
| `occupancy_max_z` | double | 100.0 | Maximum accepted Z. Narrow with `occupancy_min_z` (e.g. +3.0) for ground robots to save memory. |
| `sensor_model.max_range` | double | -1.0 | Ray truncation distance. <0 means use sensor-reported range. Set (e.g. 15.0) to cap freespace carving and speed updates. |
| `sensor_model.hit` | double (0–1) | 0.7 | Probability for an occupied endpoint (log-odds increment ≈ +0.847). Higher commits obstacles faster but can lock in noise. Typical 0.65–0.75. |
| `sensor_model.miss` | double (0–1) | 0.4 | Probability for free cells along a ray (<0.5) (log-odds ≈ -0.405). Lower (0.35) clears faster; higher (0.45) preserves thin objects. |
| `sensor_model.min` | double (0–1) | 0.12 | Lower probability clamp (log-odds ≈ -1.99). Prevents voxels from becoming irreversibly “free”. |
| `sensor_model.max` | double (0–1) | 0.97 | Upper probability clamp (log-odds ≈ +3.48). Prevents permanent saturation so later freespace can still clear. |
| `latch` | bool | false | If true, last published map message is latched (late RViz subscribers get immediate state). |
| `filter.confirm_hits` | int ≥1 | 2 | Required distinct frames a new occupied voxel must be seen before insertion. 1 disables temporal filtering. |
| `filter.confirm_window` | int (frames) | 30 | Frame window for accumulating confirmations (~3 s @ 10 Hz). Increase for intermittent visibility; decrease for faster emergence. |

Clamping: probabilities are internally converted to log-odds. `hit` adds, `miss` subtracts; `min`/`max` cap the resulting probability enabling state reversibility.

Temporal filtering details and strategy are expanded below in [Filtering "Flying" Points (Temporal Confirmation)](#filtering-flying-points-temporal-confirmation). The table above serves as a quick reference.

Example (defaults):

```yaml
bonxai_server_node:
  ros__parameters:
    resolution: 0.02
    frame_id: "map"
    base_frame_id: "base_footprint"
    occupancy_min_z: -100.0
    occupancy_max_z: 100.0
    sensor_model:
      max_range: -1.0
      hit: 0.7
      miss: 0.4
      min: 0.12
      max: 0.97
    latch: false
    filter:
      confirm_hits: 2
      confirm_window: 30
```

Quick tuning hints:

* Phantom obstacles: lower `hit` (0.65) or raise `confirm_hits` (3).
* Obstacles appear slowly: reduce `confirm_hits` (1) or slightly raise `hit`.
* Free space lingers: lower `miss` (0.35) or reduce `sensor_model.max` marginally.
* Thin poles vanish: raise `miss` (0.45) and keep `confirm_hits` ≤2.

## Filtering "Flying" Points (Temporal Confirmation)

Real-world depth and LiDAR sensors sometimes produce spurious, isolated returns ("flying" points) caused by multi‑path reflections, dust, rain, or rolling shutter artifacts. Without mitigation, a single noisy hit may create a lone occupied voxel that is never revisited and thus persists indefinitely in the map.

Bonxai provides a **minimal, low‑overhead temporal confirmation filter** to suppress these artifacts before they enter the permanent occupancy grid. The mechanism is intentionally simple to keep performance high and configuration surface small.

### How it Works

1. When a new hit falls into a voxel that is not yet allocated, it is placed into a small in‑memory **staging buffer** instead of immediately creating the voxel.
2. Each subsequent hit to the same (discretized) coordinate increments a counter.
3. If the counter reaches `confirm_hits` **within** a rolling frame window of length `confirm_window`, the voxel is promoted (allocated + standard occupancy update).
4. If the required number of hits is **not** reached before `confirm_window` frames elapse, the staged candidate is discarded silently.

If you set `confirm_hits = 1`, staging is bypassed and the filter is effectively disabled (previous behaviour).

### Choosing Values

Rule of thumb: use the *smallest* numbers that reliably suppress your noise.

Suggested starting points:

| Environment / Sensor | `confirm_hits` | `confirm_window` | Rationale |
|-----------------------|----------------|------------------|-----------|
| Indoor RGB-D / structured light | 2 | 30 | Light random speckle; fast confirmation. |
| 16–32 beam LiDAR (moderate noise) | 2 | 30 | Occasional stray beams. |
| 64+ beam LiDAR outdoors (rain / fog) | 3 | 40–60 | More transient clutter. |
| Dusty construction / mining | 3–4 | 50–80 | Frequent spurious particles. |

`confirm_window` in seconds ≈ `confirm_window / sensor_fps`. For a 10 Hz sensor and `confirm_window = 30`, candidates have up to ~3 seconds to gather the needed confirmations.

### Performance Impact

The staging buffer only holds candidates awaiting confirmation. Its size is typically a tiny fraction of confirmed voxels. Worst case (heavy, transient clutter) it is bounded by: unique_new_voxels_per_window. Each staged entry is a small struct (counter + first/last frame indices). Memory overhead is usually negligible relative to the voxel grid itself.

### When to Increase Values

- You still see isolated single voxels persisting that correspond to clear sensor noise.
- The robot sees many one‑off reflections from glass or wet surfaces.

### When to Decrease Values

- Thin structures (e.g. wires, masts) are missing until the robot pauses.
- Fast motion causes legitimate surfaces to be observed only briefly.

### Notes

- The filter only gates creation of *new* occupied voxels. Once a voxel exists, ordinary hit/miss log-odds updates apply—no additional confirmation needed.
- Free space updates (ray casting) still proceed normally through regions containing staged points; staging does not block clearing operations.
- There is deliberately no spatial neighbor check or connected-component pruning to keep CPU cost minimal; if you later need stronger structural filtering, that can be layered externally.

