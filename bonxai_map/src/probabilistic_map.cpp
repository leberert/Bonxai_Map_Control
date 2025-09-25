#include "bonxai_map/probabilistic_map.hpp"

#include <eigen3/Eigen/Geometry>
#include <unordered_set>

namespace Bonxai {

const int32_t ProbabilisticMap::UnknownProbability = ProbabilisticMap::logods(0.5f);

VoxelGrid<ProbabilisticMap::CellT>& ProbabilisticMap::grid() {
  return _grid;
}

ProbabilisticMap::ProbabilisticMap(double resolution)
    : _grid(resolution),
      _accessor(_grid.createAccessor()) {}

const VoxelGrid<ProbabilisticMap::CellT>& ProbabilisticMap::grid() const {
  return _grid;
}

const ProbabilisticMap::Options& ProbabilisticMap::options() const {
  return _options;
}

void ProbabilisticMap::setOptions(const Options& options) {
  _options = options;
}

void ProbabilisticMap::addHitPoint(const Vector3D& point) {
  const auto coord = _grid.posToCoord(point);
  CellT* cell = _accessor.value(coord, true);
  // If already updated this cycle, skip (prevents double counting)
  if (cell->update_id == _update_count) {
    return;
  }

  const bool already_occupied = cell->probability_log > _options.occupancy_threshold_log;

  // Fast path: no temporal filtering requested
  if (_options.required_observations <= 1 || _options.window_frames <= 1) {
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
  // We only keep at most 64 frames of history.
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
  { uint64_t tmp = st.mask; uint8_t c=0; while(tmp){ tmp &= (tmp-1); ++c; } st.popcnt=c; }
#endif

  // Fractional accumulation (optional)
  if (_options.fractional_hits) {
    // Add a fraction of prob_hit_log so cumulative pending equals one full hit after required_observations.
    st.pending_log = std::min(
        st.pending_log + (_options.prob_hit_log / std::max<int32_t>(1, _options.required_observations)),
        _options.prob_hit_log * 4); // clamp fractional accumulation to avoid runaway (heuristic)
  }

  // Spatial neighbor support (only computed when needed to avoid overhead)
  bool spatial_ok = (_options.min_neighbor_support == 0);
  if (!spatial_ok && st.popcnt >= _options.required_observations) {
    // Evaluate 6-neighborhood
    static const CoordT dirs[6] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    uint8_t support = 0;
    for (const auto & d : dirs) {
      const CoordT ncoord = coord + d;
      if (auto* ncell = _accessor.value(ncoord, false)) {
        if (ncell->probability_log > _options.occupancy_threshold_log) {
          if (++support >= _options.min_neighbor_support) { spatial_ok = true; break; }
        }
      } else {
        // If neighbor also staging and has at least 1 popcnt treat as weak support
        auto sit = _staging.find(ncoord);
        if (sit != _staging.end() && sit->second.popcnt > 0) {
          if (++support >= _options.min_neighbor_support) { spatial_ok = true; break; }
        }
      }
    }
    st.neighbor_support = support;
  }

  // Promotion criteria
  if (st.popcnt >= _options.required_observations && spatial_ok) {
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

void ProbabilisticMap::addMissPoint(const Vector3D& point) {
  const auto coord = _grid.posToCoord(point);
  CellT* cell = _accessor.value(coord, true);

  if (cell->update_id != _update_count) {
    cell->probability_log =
        std::max(cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);

    cell->update_id = _update_count;
    _miss_coords.push_back(coord);
  }
}

bool ProbabilisticMap::isOccupied(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log > _options.occupancy_threshold_log;
  }
  return false;
}

bool ProbabilisticMap::isUnknown(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log == _options.occupancy_threshold_log;
  }
  return true;
}

bool ProbabilisticMap::isFree(const CoordT& coord) const {
  if (auto* cell = _accessor.value(coord, false)) {
    return cell->probability_log < _options.occupancy_threshold_log;
  }
  return false;
}

void Bonxai::ProbabilisticMap::updateFreeCells(const Vector3D& origin) {
  auto accessor = _grid.createAccessor();

  // same as addMissPoint, but using lambda will force inlining
  auto clearPoint = [this, &accessor](const CoordT& coord) {
    CellT* cell = accessor.value(coord, true);
    if (cell->update_id != _update_count) {
      cell->probability_log =
          std::max(cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
      cell->update_id = _update_count;
    }
    return true;
  };

  const auto coord_origin = _grid.posToCoord(origin);

  for (const auto& coord_end : _hit_coords) {
    RayIterator(coord_origin, coord_end, clearPoint);
  }
  _hit_coords.clear();

  for (const auto& coord_end : _miss_coords) {
    RayIterator(coord_origin, coord_end, clearPoint);
  }
  _miss_coords.clear();

  if (++_update_count == 4) {
    _update_count = 1;
  }
  // staging timeout & stale cleanup
  if (!_staging.empty()) {
    const uint64_t stale_limit = (_options.stale_frames > 0) ? (_frame_counter - _options.stale_frames) : 0;
    for (auto it = _staging.begin(); it != _staging.end();) {
      if (_options.stale_frames > 0 && it->second.last_seen < stale_limit) {
        it = _staging.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Deoccupy decay: optionally apply a miss to cells not recently confirmed
  if (_options.deoccupy_frames > 0 && !_last_confirmed_hit.empty()) {
    const uint64_t decay_limit = (_frame_counter > _options.deoccupy_frames) ? (_frame_counter - _options.deoccupy_frames) : 0;
    for (auto it = _last_confirmed_hit.begin(); it != _last_confirmed_hit.end();) {
      if (it->second < decay_limit) {
        if (auto* cell = _accessor.value(it->first, false)) {
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

void ProbabilisticMap::getOccupiedVoxels(std::vector<CoordT>& coords) {
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log > _options.occupancy_threshold_log) {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

void ProbabilisticMap::getFreeVoxels(std::vector<CoordT>& coords) {
  coords.clear();
  auto visitor = [&](CellT& cell, const CoordT& coord) {
    if (cell.probability_log < _options.occupancy_threshold_log) {
      coords.push_back(coord);
    }
  };
  _grid.forEachCell(visitor);
}

}  // namespace Bonxai
