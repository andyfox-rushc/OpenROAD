// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

// Generator Code Begin Cpp
#include "dbGCellGrid.h"

#include <cstdint>
#include <cstring>
#include <map>

#include "dbDatabase.h"
#include "dbHashTable.h"
#include "dbTable.h"
#include "dbTable.hpp"
#include "dbTechLayer.h"
#include "odb/db.h"
// User Code Begin Includes
#include <algorithm>

#include "dbBlock.h"
#include "dbTech.h"
#include "odb/dbSet.h"
// User Code End Includes
namespace odb {
template class dbTable<_dbGCellGrid>;

bool _dbGCellGrid::operator==(const _dbGCellGrid& rhs) const
{
  if (flags_.x_grid_valid_ != rhs.flags_.x_grid_valid_) {
    return false;
  }
  if (flags_.y_grid_valid_ != rhs.flags_.y_grid_valid_) {
    return false;
  }

  // User Code Begin ==
  if (x_origin_ != rhs.x_origin_) {
    return false;
  }

  if (x_count_ != rhs.x_count_) {
    return false;
  }

  if (x_step_ != rhs.x_step_) {
    return false;
  }

  if (y_origin_ != rhs.y_origin_) {
    return false;
  }

  if (y_count_ != rhs.y_count_) {
    return false;
  }

  if (y_step_ != rhs.y_step_) {
    return false;
  }
  // User Code End ==
  return true;
}

bool _dbGCellGrid::operator<(const _dbGCellGrid& rhs) const
{
  // User Code Begin <
  if (getOID() >= rhs.getOID()) {
    return false;
  }
  // User Code End <
  return true;
}

_dbGCellGrid::_dbGCellGrid(_dbDatabase* db)
{
  flags_ = {};
}

dbIStream& operator>>(dbIStream& stream, _dbGCellGrid& obj)
{
  uint32_t flags_bit_field;
  stream >> flags_bit_field;
  static_assert(sizeof(obj.flags_) == sizeof(flags_bit_field));
  std::memcpy(&obj.flags_, &flags_bit_field, sizeof(flags_bit_field));
  stream >> obj.x_origin_;
  stream >> obj.x_count_;
  stream >> obj.x_step_;
  stream >> obj.y_origin_;
  stream >> obj.y_count_;
  stream >> obj.y_step_;
  stream >> obj.x_grid_;
  stream >> obj.y_grid_;
  // User Code Begin >>
  _dbDatabase* db = obj.getDatabase();
  if (db->isSchema(db_schema_gcell_grid_matrix)) {
    stream >> obj.congestion_map_;
  } else {
    std::map<dbId<_dbTechLayer>,
             std::map<std::pair<uint, uint>, dbGCellGrid::GCellData>>
        old_format;
    stream >> old_format;
    for (const auto& [lid, cells] : old_format) {
      for (const auto& [coord, data] : cells) {
        obj.get(lid)(coord.first, coord.second) = data;
      }
    }
  }
  // User Code End >>
  return stream;
}

dbOStream& operator<<(dbOStream& stream, const dbGCellGrid::GCellData& obj)
{
  stream << obj.usage;
  stream << obj.capacity;
  return stream;
}
dbOStream& operator<<(dbOStream& stream, const _dbGCellGrid& obj)
{
  uint32_t flags_bit_field;
  static_assert(sizeof(obj.flags_) == sizeof(flags_bit_field));
  std::memcpy(&flags_bit_field, &obj.flags_, sizeof(obj.flags_));
  stream << flags_bit_field;
  stream << obj.x_origin_;
  stream << obj.x_count_;
  stream << obj.x_step_;
  stream << obj.y_origin_;
  stream << obj.y_count_;
  stream << obj.y_step_;
  stream << obj.x_grid_;
  stream << obj.y_grid_;
  // User Code Begin <<
  stream << obj.congestion_map_;
  // User Code End <<
  return stream;
}

void _dbGCellGrid::collectMemInfo(MemInfo& info)
{
  info.cnt++;
  info.size += sizeof(*this);

  // User Code Begin collectMemInfo
  info.children_["x_origin"].add(x_origin_);
  info.children_["x_count_"].add(x_count_);
  info.children_["x_step_"].add(x_step_);
  info.children_["y_origin_"].add(y_origin_);
  info.children_["y_count_"].add(y_count_);
  info.children_["y_step_"].add(y_step_);
  info.children_["x_grid_"].add(x_grid_);
  info.children_["y_grid_"].add(y_grid_);

  MemInfo& congestion_info = info.children_["congestion"];
  for (auto& [layer, data] : congestion_map_) {
    congestion_info.add(data);
  }
  // User Code End collectMemInfo
}

// User Code Begin PrivateMethods

dbIStream& operator>>(dbIStream& stream, dbGCellGrid::GCellData& obj)
{
  if (stream.getDatabase()->isSchema(db_schema_smaler_gcelldata)) {
    stream >> obj.usage;
    stream >> obj.capacity;
  } else {
    uint horizontal_usage;
    uint vertical_usage;
    uint up_usage;
    uint horizontal_capacity;
    uint vertical_capacity;
    uint up_capacity;

    stream >> horizontal_usage;
    stream >> vertical_usage;
    stream >> up_usage;
    stream >> horizontal_capacity;
    stream >> vertical_capacity;
    stream >> up_capacity;

    obj.usage = vertical_usage + horizontal_usage + up_usage;
    obj.capacity = horizontal_capacity + vertical_capacity + up_capacity;
  }
  return stream;
}

dbMatrix<dbGCellGrid::GCellData>& _dbGCellGrid::get(
    const dbId<_dbTechLayer>& lid)
{
  if (congestion_map_.empty()) {
    dbGCellGrid* pub_grid = (dbGCellGrid*) this;
    std::vector<int> grid;
    pub_grid->getGridX(grid);
    const uint num_x = grid.size();
    pub_grid->getGridY(grid);
    const uint num_y = grid.size();

    dbMatrix<dbGCellGrid::GCellData> data(num_x, num_y);
    auto [iter, ins]
        = congestion_map_.emplace(std::make_pair(lid, std::move(data)));
    return iter->second;
  }
  auto it = congestion_map_.find(lid);
  if (it != congestion_map_.end()) {
    return it->second;
  }

  it = congestion_map_.begin();
  const uint num_rows = it->second.numRows();
  const uint num_cols = it->second.numCols();

  dbMatrix<dbGCellGrid::GCellData> data(num_rows, num_cols);
  auto [iter, ins]
      = congestion_map_.emplace(std::make_pair(lid, std::move(data)));
  return iter->second;
}

dbTechLayer* _dbGCellGrid::getLayer(const dbId<_dbTechLayer>& lid) const
{
  _dbGCellGrid* obj = (_dbGCellGrid*) this;
  dbDatabase* db = (dbDatabase*) obj->getDatabase();
  _dbTech* tech = (_dbTech*) db->getTech();
  return (dbTechLayer*) tech->_layer_tbl->getPtr(lid);
}

// User Code End PrivateMethods

////////////////////////////////////////////////////////////////////
//
// dbGCellGrid - Methods
//
////////////////////////////////////////////////////////////////////

// User Code Begin dbGCellGridPublicMethods

void dbGCellGrid::getGridX(std::vector<int>& x_grid)
{
  _dbGCellGrid* grid = (_dbGCellGrid*) this;
  if (grid->flags_.x_grid_valid_) {
    x_grid = grid->x_grid_;
    return;
  }
  grid->x_grid_.clear();
  uint i;
  for (i = 0; i < grid->x_origin_.size(); ++i) {
    int j;

    int x = grid->x_origin_[i];
    int count = grid->x_count_[i];
    int step = grid->x_step_[i];

    for (j = 0; j < count; ++j) {
      grid->x_grid_.push_back(x);
      x += step;
    }
  }
  grid->flags_.x_grid_valid_ = true;
  // empty grid
  if (grid->x_grid_.begin() == grid->x_grid_.end()) {
    x_grid = grid->x_grid_;
    return;
  }

  // sort coords in asscending order
  std::sort(grid->x_grid_.begin(), grid->x_grid_.end());

  // remove any duplicates
  auto new_end = std::unique(grid->x_grid_.begin(), grid->x_grid_.end());
  grid->x_grid_.erase(new_end, grid->x_grid_.end());
  x_grid = grid->x_grid_;
}

void dbGCellGrid::getGridY(std::vector<int>& y_grid)
{
  _dbGCellGrid* grid = (_dbGCellGrid*) this;
  if (grid->flags_.y_grid_valid_) {
    y_grid = grid->y_grid_;
    return;
  }
  grid->y_grid_.clear();

  uint i;

  for (i = 0; i < grid->y_origin_.size(); ++i) {
    int j;

    int y = grid->y_origin_[i];
    int count = grid->y_count_[i];
    int step = grid->y_step_[i];

    for (j = 0; j < count; ++j) {
      grid->y_grid_.push_back(y);
      y += step;
    }
  }
  grid->flags_.y_grid_valid_ = true;
  // empty grid
  if (grid->y_grid_.begin() == grid->y_grid_.end()) {
    y_grid = grid->y_grid_;
    return;
  }

  // sort coords in asscending order
  std::sort(grid->y_grid_.begin(), grid->y_grid_.end());

  // remove any duplicates
  auto new_end = std::unique(grid->y_grid_.begin(), grid->y_grid_.end());
  grid->y_grid_.erase(new_end, grid->y_grid_.end());
  y_grid = grid->y_grid_;
}

dbBlock* dbGCellGrid::getBlock()
{
  return (dbBlock*) getImpl()->getOwner();
}

void dbGCellGrid::addGridPatternX(int origin_x, int line_count, int step)
{
  _dbGCellGrid* grid = (_dbGCellGrid*) this;
  grid->x_origin_.push_back(origin_x);
  grid->x_count_.push_back(line_count);
  grid->x_step_.push_back(step);
  grid->flags_.x_grid_valid_ = false;
  resetCongestionMap();
}

void dbGCellGrid::addGridPatternY(int origin_y, int line_count, int step)
{
  _dbGCellGrid* grid = (_dbGCellGrid*) this;
  grid->y_origin_.push_back(origin_y);
  grid->y_count_.push_back(line_count);
  grid->y_step_.push_back(step);
  grid->flags_.y_grid_valid_ = false;
  resetCongestionMap();
}

int dbGCellGrid::getNumGridPatternsX()
{
  _dbGCellGrid* grid = (_dbGCellGrid*) this;
  return grid->x_origin_.size();
}

int dbGCellGrid::getNumGridPatternsY()
{
  _dbGCellGrid* grid = (_dbGCellGrid*) this;
  return grid->y_origin_.size();
}

void dbGCellGrid::getGridPatternX(int i,
                                  int& origin_x,
                                  int& line_count,
                                  int& step)
{
  _dbGCellGrid* grid = (_dbGCellGrid*) this;
  ZASSERT(i < (int) grid->x_origin_.size());
  origin_x = grid->x_origin_[i];
  line_count = grid->x_count_[i];
  step = grid->x_step_[i];
}

void dbGCellGrid::getGridPatternY(int i,
                                  int& origin_y,
                                  int& line_count,
                                  int& step)
{
  _dbGCellGrid* grid = (_dbGCellGrid*) this;
  ZASSERT(i < (int) grid->y_origin_.size());
  origin_y = grid->y_origin_[i];
  line_count = grid->y_count_[i];
  step = grid->y_step_[i];
}

dbGCellGrid* dbGCellGrid::create(dbBlock* block_)
{
  _dbBlock* block = (_dbBlock*) block_;

  if (block->_gcell_grid != 0) {
    return nullptr;
  }

  _dbGCellGrid* grid = block->_gcell_grid_tbl->create();
  block->_gcell_grid = grid->getOID();
  return (dbGCellGrid*) grid;
}

dbGCellGrid* dbGCellGrid::getGCellGrid(dbBlock* block_, uint dbid_)
{
  _dbBlock* block = (_dbBlock*) block_;
  return (dbGCellGrid*) block->_gcell_grid_tbl->getPtr(dbid_);
}

uint dbGCellGrid::getXIdx(int x)
{
  std::vector<int> grid;
  getGridX(grid);
  if (grid.empty() || grid[0] > x) {
    return 0;
  }
  auto pos = --(std::upper_bound(grid.begin(), grid.end(), x));
  return (int) std::distance(grid.begin(), pos);
}

uint dbGCellGrid::getYIdx(int y)
{
  std::vector<int> grid;
  getGridY(grid);
  if (grid.empty() || grid[0] > y) {
    return 0;
  }
  auto pos = --(std::upper_bound(grid.begin(), grid.end(), y));
  return (int) std::distance(grid.begin(), pos);
}

uint8_t dbGCellGrid::getCapacity(dbTechLayer* layer,
                                 uint x_idx,
                                 uint y_idx) const
{
  _dbGCellGrid* _grid = (_dbGCellGrid*) this;
  uint lid = layer->getId();
  return _grid->get(lid)(x_idx, y_idx).capacity;
}

uint8_t dbGCellGrid::getUsage(dbTechLayer* layer, uint x_idx, uint y_idx) const
{
  _dbGCellGrid* _grid = (_dbGCellGrid*) this;
  uint lid = layer->getId();
  return _grid->get(lid)(x_idx, y_idx).usage;
}

void dbGCellGrid::setCapacity(dbTechLayer* layer,
                              uint x_idx,
                              uint y_idx,
                              uint8_t capacity)
{
  _dbGCellGrid* _grid = (_dbGCellGrid*) this;
  uint lid = layer->getId();
  _grid->get(lid)(x_idx, y_idx).capacity = capacity;
}

void dbGCellGrid::setUsage(dbTechLayer* layer,
                           uint x_idx,
                           uint y_idx,
                           uint8_t use)
{
  _dbGCellGrid* _grid = (_dbGCellGrid*) this;
  uint lid = layer->getId();
  _grid->get(lid)(x_idx, y_idx).usage = use;
}

void dbGCellGrid::resetCongestionMap()
{
  _dbGCellGrid* _grid = (_dbGCellGrid*) this;
  _grid->congestion_map_.clear();
}

void dbGCellGrid::resetGrid()
{
  _dbGCellGrid* _grid = (_dbGCellGrid*) this;
  _grid->x_origin_.clear();
  _grid->x_count_.clear();
  _grid->x_step_.clear();
  _grid->y_origin_.clear();
  _grid->y_count_.clear();
  _grid->y_step_.clear();
  _grid->x_grid_.clear();
  _grid->y_grid_.clear();
  _grid->congestion_map_.clear();
  _grid->flags_.x_grid_valid_ = true;
  _grid->flags_.y_grid_valid_ = true;
}

dbMatrix<dbGCellGrid::GCellData> dbGCellGrid::getLayerCongestionMap(
    dbTechLayer* layer)
{
  _dbGCellGrid* _grid = (_dbGCellGrid*) this;
  const auto& cmap = _grid->congestion_map_;
  if (cmap.find(layer->getId()) != cmap.end()) {
    return cmap.at(layer->getId());
  }
  return {};
}

dbMatrix<dbGCellGrid::GCellData> dbGCellGrid::getDirectionCongestionMap(
    const dbTechLayerDir& direction)
{
  _dbGCellGrid* _grid = (_dbGCellGrid*) this;
  const auto& cmap = _grid->congestion_map_;
  auto iter = cmap.begin();
  if (iter == cmap.end()) {
    return {};
  }
  const int num_rows = iter->second.numRows();
  const int num_cols = iter->second.numCols();
  dbMatrix<dbGCellGrid::GCellData> congestion(num_rows, num_cols);
  for (auto& [lid, matrix] : cmap) {
    dbTechLayer* tech_layer = _grid->getLayer(lid);
    if (direction == tech_layer->getDirection()) {
      for (int row = 0; row < num_rows; ++row) {
        for (int col = 0; col < num_cols; ++col) {
          congestion(row, col).usage += matrix(row, col).usage;
          congestion(row, col).capacity += matrix(row, col).capacity;
        }
      }
    }
  }
  return congestion;
}
// User Code End dbGCellGridPublicMethods
}  // namespace odb
// Generator Code End Cpp
