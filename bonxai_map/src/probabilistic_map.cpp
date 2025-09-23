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

  if (already_occupied || _options.confirm_hits <= 1) {
    cell->probability_log =
        std::min(cell->probability_log + _options.prob_hit_log, _options.clamp_max_log);
    cell->update_id = _update_count;
    _hit_coords.push_back(coord);
    return;
  }

  // staged confirmation path
  auto& st = _staging[coord];
  if (st.hits == 0) {
    st.last_seen = _frame_counter;  // first observation
  }
  st.hits++;
  st.last_seen = _frame_counter;
  if (st.hits >= _options.confirm_hits) {
    _staging.erase(coord);
    cell->probability_log =
        std::min(cell->probability_log + _options.prob_hit_log, _options.clamp_max_log);
    cell->update_id = _update_count;
    _hit_coords.push_back(coord);
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
  // staging timeout cleanup (optional)
  if (!_staging.empty() && _options.confirm_window > 0) {
    const uint64_t frame_limit = _frame_counter - _options.confirm_window;
    for (auto it = _staging.begin(); it != _staging.end();) {
      if (it->second.last_seen < frame_limit) {
        it = _staging.erase(it);
      } else {
        ++it;
      }
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
