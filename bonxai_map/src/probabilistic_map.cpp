#include "bonxai_map/probabilistic_map.hpp"

#include <eigen3/Eigen/Geometry>
#include <unordered_set>

namespace Bonxai
{

const int32_t ProbabilisticMap::UnknownProbability = ProbabilisticMap::logods(0.5f);

VoxelGrid<ProbabilisticMap::CellT> & ProbabilisticMap::grid()
{
  return _grid;
}

ProbabilisticMap::ProbabilisticMap(double resolution)
: _grid(resolution),
  _accessor(_grid.createAccessor()) {}

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
    // Add a fraction of prob_hit_log so cumulative pending equals one full hit after required_observations.
    st.pending_log = std::min(
      st.pending_log + (_options.prob_hit_log / std::max<int32_t>(1, _options.required_observations)),
      _options.prob_hit_log * 4);   // clamp fractional accumulation to avoid runaway (heuristic)
  }

  // Enhanced spatial neighbor support with noise filtering
  bool spatial_ok = (_options.min_neighbor_support == 0);
  if (!spatial_ok && st.popcnt >= _options.required_observations) {
    // Evaluate 6-connectivity + extended neighborhood for isolated points
    static const CoordT dirs_6[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    uint8_t support = 0;
    uint8_t staging_support = 0;

    // Create a new accessor for neighbor checks to avoid potential corruption
    auto neighbor_accessor = _grid.createAccessor();

    // First check 6-connectivity for standard neighbor support
    for (const auto & d : dirs_6) {
      const CoordT ncoord = coord + d;

      if (auto * ncell = neighbor_accessor.value(ncoord, false)) {
        if (ncell->probability_log > _options.occupancy_threshold_log) {
          support++;
        }
      } else {
        // Check staging neighbors but be more restrictive
        auto sit = _staging.find(ncoord);
        if (sit != _staging.end() && sit->second.popcnt >= _options.required_observations) {
          staging_support++;
        }
      }
    }

    // For potentially isolated noise points (especially elevated ones), apply stricter criteria
    const double point_z = _grid.coordToPos(coord).z;
    bool is_elevated = point_z > 1.5; // Above 1.5m is considered elevated

    if (is_elevated && support == 0 && staging_support == 0) {
      // For elevated isolated points, check extended 26-connectivity neighborhood
      static const CoordT dirs_26[26] = {
        // Face neighbors (6)
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
        // Edge neighbors (12)
        {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
        {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
        {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1},
        // Corner neighbors (8)
        {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
        {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}
      };

      uint8_t extended_support = 0;
      uint8_t extended_staging = 0;

      for (const auto & d : dirs_26) {
        const CoordT ncoord = coord + d;

        if (auto * ncell = neighbor_accessor.value(ncoord, false)) {
          if (ncell->probability_log > _options.occupancy_threshold_log) {
            extended_support++;
          }
        } else {
          auto sit = _staging.find(ncoord);
          if (sit != _staging.end() && sit->second.popcnt >= _options.required_observations) {
            extended_staging++;
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
    const double point_z = _grid.coordToPos(coord).z;
    bool is_elevated = point_z > 1.5; // Above 1.5m is considered elevated

    // Calculate minimum persistence threshold based on required_observations and elevation
    uint8_t persistence_threshold;
    if (is_elevated) {
      // Elevated points need more evidence regardless of required_observations
      persistence_threshold = std::max<uint8_t>(2, _options.required_observations);
    } else {
      // Ground-level points: allow promotion with required observations even without spatial support
      // This breaks the chicken-and-egg problem for fresh maps
      persistence_threshold = _options.required_observations;
    }

    if (st.popcnt >= persistence_threshold) {
      spatial_ok = true;
    }
  }

  if (st.popcnt >= std::max<uint8_t>(1, _options.required_observations) && spatial_ok) {
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
  auto accessor = _grid.createAccessor();

  // same as addMissPoint, but using lambda will force inlining
  auto clearPoint = [this, &accessor](const CoordT & coord) {
      CellT * cell = accessor.value(coord, true);
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

        if (point_z > 1.5) { // Elevated point
          // Check if it's still isolated
          static const CoordT dirs[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
          uint8_t neighbor_count = 0;

          for (const auto & d : dirs) {
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
