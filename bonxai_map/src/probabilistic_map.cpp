#include "bonxai_map/probabilistic_map.hpp"

#include <eigen3/Eigen/Geometry>
#include <unordered_set>
#include <array>

namespace Bonxai
{

const int32_t ProbabilisticMap::UnknownProbability = ProbabilisticMap::logods(0.5f);

// Spatial Neighbor Lookup Optimization (#2): Pre-computed static neighbor direction arrays
const std::array<CoordT, ProbabilisticMap::MAX_NEIGHBORS_6> ProbabilisticMap::NEIGHBOR_DIRS_6 = {{
  {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
}};

const std::array<CoordT, ProbabilisticMap::MAX_NEIGHBORS_26> ProbabilisticMap::NEIGHBOR_DIRS_26 = {{
  // Face neighbors (6)
  {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
  // Edge neighbors (12)
  {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
  {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
  {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1},
  // Corner neighbors (8)
  {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
  {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}
}};

// Thread-local storage definitions for neighbor lookup caches
thread_local std::vector<CoordT> ProbabilisticMap::_neighbor_coords_cache;
thread_local std::vector<const ProbabilisticMap::CellT *> ProbabilisticMap::_neighbor_cells_cache;

VoxelGrid<ProbabilisticMap::CellT> & ProbabilisticMap::grid()
{
  return _grid;
}

ProbabilisticMap::ProbabilisticMap(double resolution)
: _grid(resolution),
  _accessor(_grid.createAccessor())
{
  // Thread-local caches are initialized automatically when first accessed
}

const VoxelGrid<ProbabilisticMap::CellT> & ProbabilisticMap::grid() const
{
  return _grid;
}

const ProbabilisticMap::Options & ProbabilisticMap::options() const
{
  return _options;
}

void ProbabilisticMap::setOptions(const Options & options)
{
  _options = options;
}

void ProbabilisticMap::addHitPoint(const Vector3D & point)
{
  const auto coord = _grid.posToCoord(point);
  CellT * cell = _accessor.value(coord, true);

  if (!cell) {
    return; // Skip this point if we can't create/access the cell
  }

  // If already updated this cycle, skip (prevents double counting)
  if (cell->update_id == _update_count) {
    return;
  }

  const bool already_occupied = cell->probability_log > _options.occupancy_threshold_log;

  // Fast path: immediate integration when temporal filtering effectively disabled.
  if (_options.min_neighbor_support == 0 &&
    (_options.required_observations <= 1 || _options.window_frames <= 1))
  {
    cell->probability_log = std::min(cell->probability_log + _options.prob_hit_log, _options.clamp_max_log);
    cell->update_id = _update_count;
    _hit_coords.push_back(coord);
    _last_confirmed_hit[coord] = static_cast<uint32_t>(_frame_counter);
    return;
  }

  if (already_occupied) {
    // Update occupancy probability normally and refresh last_confirmed_hit
    cell->probability_log = std::min(cell->probability_log + _options.prob_hit_log, _options.clamp_max_log);
    cell->update_id = _update_count;
    _hit_coords.push_back(coord);
    _last_confirmed_hit[coord] = static_cast<uint32_t>(_frame_counter);
    return;
  }

  // Staging path with temporal bitmask
  auto & st = _staging[coord];
  if (st.first_seen == 0) {
    st.first_seen = _frame_counter;
  }
  st.last_seen = _frame_counter;

  // Shift mask left by one, insert new observation bit at LSB position.
  if (_options.window_frames >= 64) {
    st.mask = (st.mask << 1) | 1ULL;
  } else {
    const uint64_t mask_limit = (1ULL << _options.window_frames) - 1ULL;
    st.mask = ((st.mask << 1) | 1ULL) & mask_limit;
  }

  // Recompute population count (can use builtin if available)
#if defined(__GNUG__)
  st.popcnt = static_cast<uint8_t>(__builtin_popcountll(st.mask));
#else
  {uint64_t tmp = st.mask; uint8_t c = 0; while (tmp) {tmp &= (tmp - 1); ++c;} st.popcnt = c;}
#endif

  // Fractional accumulation (optional)
  if (_options.fractional_hits) {
    // Effective required observations for fractional accumulation can differ for elevated points under
    // the semantic override of min_neighbor_support==2 (fixed 2 observations).
    int32_t effective_required = std::max<int32_t>(1, _options.required_observations);
    const double tmp_point_z = _grid.coordToPos(coord).z; // temp to avoid recomputing later if not cached
    if (tmp_point_z > 2.0 && _options.min_neighbor_support == 2) {
      // Elevated strict mode: require one extra observation beyond required_observations
      effective_required = std::max<int32_t>(2, _options.required_observations + 1);
    }
    st.pending_log = std::min(
      st.pending_log + (_options.prob_hit_log / effective_required),
      _options.prob_hit_log * 4);   // clamp fractional accumulation to avoid runaway (heuristic)
  }

  // Determine elevation status once (used by both spatial and temporal logic)
  const double point_z_current = _grid.coordToPos(coord).z;
  const bool is_elevated = point_z_current > 2.0; // Above threshold considered elevated


  // Enhanced spatial neighbor support with noise filtering - OPTIMIZED VERSION
  bool spatial_ok = (_options.min_neighbor_support == 0);
  // Spatial neighbor evaluation gate: normally waits for required_observations.
  // For min_neighbor_support==2 and elevated, require (required_observations + 1) observations before spatial test.
  // For min_neighbor_support==1 and elevated we allow spatial test as soon as we have 1 observation (popcnt>=1).
  uint8_t spatial_obs_gate = _options.required_observations;
  if (is_elevated) {
    if (_options.min_neighbor_support == 1) {
      spatial_obs_gate = 1; // evaluate neighbors early to allow immediate spatial confirmation
    } else if (_options.min_neighbor_support == 2) {
      // Strict elevated mode: need required_observations + 1 frames before neighbor evaluation
      spatial_obs_gate = std::max<uint8_t>(2, static_cast<uint8_t>(_options.required_observations + 1));
    }
  }

  if (!spatial_ok && st.popcnt >= spatial_obs_gate) {
    uint8_t support = 0;
    uint8_t staging_support = 0;

    // OPTIMIZATION: Use pre-allocated thread-local caches to avoid repeated allocations
    _neighbor_coords_cache.clear();
    _neighbor_cells_cache.clear();

    // Ensure caches have sufficient capacity (initialize on first use)
    if (_neighbor_coords_cache.capacity() < NEIGHBOR_DIRS_26.size()) {
      _neighbor_coords_cache.reserve(NEIGHBOR_DIRS_26.size());
    }
    if (_neighbor_cells_cache.capacity() < NEIGHBOR_DIRS_26.size()) {
      _neighbor_cells_cache.reserve(NEIGHBOR_DIRS_26.size());
    }

    // Pre-compute all 6-connectivity neighbor coordinates
    for (const auto & d : NEIGHBOR_DIRS_6) {
      _neighbor_coords_cache.push_back(coord + d);
    }

    // OPTIMIZATION: Batch lookup neighbor cells to reduce hash map overhead
    for (const auto & ncoord : _neighbor_coords_cache) {
      _neighbor_cells_cache.push_back(_accessor.value(ncoord, false));
    }

    // Process neighbor cells and staging in single optimized pass
    for (size_t i = 0; i < _neighbor_coords_cache.size(); ++i) {
      const auto & ncoord = _neighbor_coords_cache[i];
      const auto * ncell = _neighbor_cells_cache[i];

      if (ncell) {
        if (ncell->probability_log > _options.occupancy_threshold_log) {
          support++;
        }
      } else {
        // OPTIMIZATION: Use find() instead of operator[] to avoid unnecessary insertions
        {
          auto sit = _staging.find(ncoord);
          if (sit != _staging.end() && sit->second.popcnt >= _options.required_observations) {
            staging_support++;
          }
        }
      }
    }

    // For potentially isolated noise points (especially elevated ones), apply stricter criteria
    if (is_elevated && support == 0 && staging_support == 0) {
      // OPTIMIZATION: For elevated isolated points, check extended 26-connectivity neighborhood
      uint8_t extended_support = 0;
      uint8_t extended_staging = 0;

      // OPTIMIZATION: Reuse caches, clear and resize for 26-connectivity
      _neighbor_coords_cache.clear();
      _neighbor_cells_cache.clear();
      _neighbor_coords_cache.reserve(NEIGHBOR_DIRS_26.size());
      _neighbor_cells_cache.reserve(NEIGHBOR_DIRS_26.size());

      // Pre-compute all 26-connectivity neighbor coordinates using static array
      for (const auto & d : NEIGHBOR_DIRS_26) {
        _neighbor_coords_cache.push_back(coord + d);
      }

      // OPTIMIZATION: Batch lookup all neighbor cells using member accessor
      for (const auto & ncoord : _neighbor_coords_cache) {
        _neighbor_cells_cache.push_back(_accessor.value(ncoord, false));
      }

      // Process all neighbors in single optimized pass
      for (size_t i = 0; i < _neighbor_coords_cache.size(); ++i) {
        const auto & ncoord = _neighbor_coords_cache[i];
        const auto * ncell = _neighbor_cells_cache[i];

        if (ncell) {
          if (ncell->probability_log > _options.occupancy_threshold_log) {
            extended_support++;
          }
        } else {
          // OPTIMIZATION: Efficient staging lookup with structured binding
          {
            auto sit = _staging.find(ncoord);
            if (sit != _staging.end() && sit->second.popcnt >= _options.required_observations) {
              extended_staging++;
            }
          }
        }
      }

      // Require at least 2 extended neighbors for elevated isolated points
      spatial_ok =
        (extended_support + extended_staging >= std::max(2u, static_cast<unsigned>(_options.min_neighbor_support)));
    } else {
      // Standard spatial support check
      spatial_ok = (support + staging_support >= _options.min_neighbor_support);
    }

    st.neighbor_support = support + staging_support;
  }

  // Enhanced temporal self-persistence with noise filtering
  if (!spatial_ok) {
    bool allow_temporal_fallback_elevated = true;
    if (is_elevated && (_options.min_neighbor_support == 1 || _options.min_neighbor_support == 2)) {
      // In new semantics, elevated points with neighbor support modes 1 or 2 must not rely solely on temporal fallback
      allow_temporal_fallback_elevated = false;
    }

    if (allow_temporal_fallback_elevated || !is_elevated) {
      // Calculate minimum persistence threshold based on required_observations and elevation
      uint8_t persistence_threshold;
      if (is_elevated) {
        // Elevated points (only if allowed) need more evidence regardless of required_observations
        persistence_threshold = std::max<uint8_t>(2, _options.required_observations);
      } else {
        // Ground-level points: allow promotion with required observations even without spatial support
        persistence_threshold = _options.required_observations;
      }

      if (st.popcnt >= persistence_threshold) {
        spatial_ok = true;
      }
    }
  }

  // Promotion gating: ensure observation threshold is met. For elevated with min_neighbor_support==2 enforce 2 obs.
  uint8_t promote_obs_gate = std::max<uint8_t>(1, _options.required_observations);
  if (is_elevated && _options.min_neighbor_support == 2) {
    // Elevated strict mode: promote only after required_observations + 1 observations
    promote_obs_gate = std::max<uint8_t>(2, static_cast<uint8_t>(_options.required_observations + 1));
  }

  if (st.popcnt >= promote_obs_gate && spatial_ok) {
    int32_t add_log = _options.fractional_hits ? st.pending_log : _options.prob_hit_log;
    add_log = std::max<int32_t>(add_log, _options.prob_hit_log); // ensure at least one full hit effect
    cell->probability_log = std::min(cell->probability_log + add_log, _options.clamp_max_log);
    cell->update_id = _update_count;
    _hit_coords.push_back(coord);
    _last_confirmed_hit[coord] = static_cast<uint32_t>(_frame_counter);
    _staging.erase(coord);
  } else {
    // Not yet promoted; do not push into _hit_coords (so free ray clearing does not originate here yet)
    cell->update_id = _update_count; // mark touched to avoid multiple staging increments this cycle
  }
}

void ProbabilisticMap::addMissPoint(const Vector3D & point)
{
  const auto coord = _grid.posToCoord(point);
  CellT * cell = _accessor.value(coord, true);

  if (cell->update_id != _update_count) {
    cell->probability_log =
      std::max(cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);

    cell->update_id = _update_count;
    _miss_coords.push_back(coord);
  }
}

bool ProbabilisticMap::isOccupied(const CoordT & coord) const
{
  if (auto * cell = _accessor.value(coord, false)) {
    return cell->probability_log > _options.occupancy_threshold_log;
  }
  return false;
}

bool ProbabilisticMap::isUnknown(const CoordT & coord) const
{
  if (auto * cell = _accessor.value(coord, false)) {
    return cell->probability_log == _options.occupancy_threshold_log;
  }
  return true;
}

bool ProbabilisticMap::isFree(const CoordT & coord) const
{
  if (auto * cell = _accessor.value(coord, false)) {
    return cell->probability_log < _options.occupancy_threshold_log;
  }
  return false;
}

void Bonxai::ProbabilisticMap::updateFreeCells(const Vector3D & origin)
{
  // OPTIMIZATION: Reuse member accessor instead of creating new one (expensive!)
  // This avoids the costly createAccessor() call every frame

  // same as addMissPoint, but using lambda will force inlining
  auto clearPoint = [this](const CoordT & coord) {
      CellT * cell = _accessor.value(coord, true);
      if (cell->update_id != _update_count) {
        cell->probability_log =
          std::max(cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
        cell->update_id = _update_count;
      }
      return true;
    };

  const auto coord_origin = _grid.posToCoord(origin);

  for (const auto & coord_end : _hit_coords) {
    RayIterator(coord_origin, coord_end, clearPoint);
  }
  _hit_coords.clear();

  for (const auto & coord_end : _miss_coords) {
    RayIterator(coord_origin, coord_end, clearPoint);
  }
  _miss_coords.clear();

  if (++_update_count == 4) {
    _update_count = 1;
  }
  // staging timeout & stale cleanup
  if (!_staging.empty()) {
    const uint64_t stale_limit = (_options.stale_frames > 0) ? (_frame_counter - _options.stale_frames) : 0;
    size_t stale_removed = 0;
    for (auto it = _staging.begin(); it != _staging.end(); ) {
      if (_options.stale_frames > 0 && it->second.last_seen < stale_limit) {
        it = _staging.erase(it);
        stale_removed++;
      } else {
        ++it;
      }
    }
  }

  // Enhanced deoccupy decay with isolated noise point removal
  if (!_last_confirmed_hit.empty()) {
    const uint64_t decay_limit = (_options.deoccupy_frames > 0 && _frame_counter > _options.deoccupy_frames) ?
      (_frame_counter - _options.deoccupy_frames) : 0;

    for (auto it = _last_confirmed_hit.begin(); it != _last_confirmed_hit.end(); ) {
      bool should_decay = (_options.deoccupy_frames > 0 && it->second < decay_limit);

      // Additional check: isolated elevated points get more aggressive decay
      if (!should_decay && _frame_counter % 10 == 0) { // Check every 10 frames
        const auto coord = it->first;
        const double point_z = _grid.coordToPos(coord).z;

        if (point_z > 2.0) { // Elevated point
          // OPTIMIZATION: Check if it's still isolated using pre-computed neighbor directions
          uint8_t neighbor_count = 0;

          for (const auto & d : NEIGHBOR_DIRS_6) {
            const CoordT ncoord = coord + d;
            if (auto * ncell = _accessor.value(ncoord, false)) {
              if (ncell->probability_log > _options.occupancy_threshold_log) {
                neighbor_count++;
              }
            }
          }

          // If elevated and isolated, apply decay even without time limit
          if (neighbor_count == 0) {
            should_decay = true;
          }
        }
      }

      if (should_decay) {
        if (auto * cell = _accessor.value(it->first, false)) {
          cell->probability_log = std::max(cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
          // If it drops below occupancy threshold we can remove tracking
          if (cell->probability_log <= _options.occupancy_threshold_log) {
            it = _last_confirmed_hit.erase(it);
            continue;
          }
        }
      }
      ++it;
    }
  }
  // After each full insertion cycle increment frame counter
  ++_frame_counter;
}

void ProbabilisticMap::getOccupiedVoxels(std::vector<CoordT> & coords)
{
  coords.clear();
  auto visitor = [&](CellT & cell, const CoordT & coord) {
      if (cell.probability_log > _options.occupancy_threshold_log) {
        coords.push_back(coord);
      }
    };
  _grid.forEachCell(visitor);
}

void ProbabilisticMap::getFreeVoxels(std::vector<CoordT> & coords)
{
  coords.clear();
  auto visitor = [&](CellT & cell, const CoordT & coord) {
      if (cell.probability_log < _options.occupancy_threshold_log) {
        coords.push_back(coord);
      }
    };
  _grid.forEachCell(visitor);
}

}  // namespace Bonxai
