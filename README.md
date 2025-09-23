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

## Filtering "Flying" Points (Temporal Confirmation)

Real-world depth and LiDAR sensors sometimes produce spurious, isolated returns ("flying" points) caused by multi‑path reflections, dust, rain, or rolling shutter artifacts. Without mitigation, a single noisy hit may create a lone occupied voxel that is never revisited and thus persists indefinitely in the map.

Bonxai provides a **minimal, low‑overhead temporal confirmation filter** to suppress these artifacts before they enter the permanent occupancy grid. The mechanism is intentionally simple to keep performance high and configuration surface small.

### How it Works

1. When a new hit falls into a voxel that is not yet allocated, it is placed into a small in‑memory **staging buffer** instead of immediately creating the voxel.
2. Each subsequent hit to the same (discretized) coordinate increments a counter.
3. If the counter reaches `confirm_hits` **within** a rolling frame window of length `confirm_window`, the voxel is promoted (allocated + standard occupancy update).
4. If the required number of hits is **not** reached before `confirm_window` frames elapse, the staged candidate is discarded silently.

If you set `confirm_hits = 1`, staging is bypassed and the filter is effectively disabled (previous behaviour).

### Parameters

YAML namespace: `filter` (see `bonxai_ros/params/bonxai_params.yaml`).

| Parameter | Type | Meaning |
|-----------|------|---------|
| `confirm_hits` | integer (>=1) | Number of independent observations required to promote a new voxel. 1 disables the filter. |
| `confirm_window` | integer (>=1) | Number of map update cycles (typically frames) within which the hits must accumulate. |

Promotion condition (for `confirm_hits > 1`):  hits_for_voxel >= confirm_hits  AND  (last_hit_frame - first_hit_frame) < confirm_window.

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

### Disabling the Filter

Set `confirm_hits: 1` (the default historically). Leave `confirm_window` unchanged; it is ignored in that mode.

### Example YAML Snippet

```yaml
filter:
  confirm_hits: 2        # require two hits before inserting
  confirm_window: 30     # must occur within 30 frames (~3 s at 10 Hz)
```

### Notes

- The filter only gates creation of *new* occupied voxels. Once a voxel exists, ordinary hit/miss log-odds updates apply—no additional confirmation needed.
- Free space updates (ray casting) still proceed normally through regions containing staged points; staging does not block clearing operations.
- There is deliberately no spatial neighbor check or connected-component pruning to keep CPU cost minimal; if you later need stronger structural filtering, that can be layered externally.

