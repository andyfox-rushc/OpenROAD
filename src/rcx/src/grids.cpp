///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2019, Nefelus Inc
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "grids.h"

#include <cstdio>

namespace rcx {

Ath__box::Ath__box()
{
  set(0, 0, 0, 0);
  _layer = 0;
}

int Ath__box::getXlo(const int bound) const
{
  return std::max(_rect.xMin(), bound);
}

int Ath__box::getYlo(const int bound) const
{
  return std::max(_rect.yMin(), bound);
}

int Ath__box::getXhi(const int bound) const
{
  return std::min(_rect.xMax(), bound);
}

int Ath__box::getYhi(const int bound) const
{
  return std::min(_rect.yMax(), bound);
}

uint Ath__box::getDir() const
{
  const uint dx = getDX();
  const uint dy = getDY();

  return (dx < dy) ? 0 : 1;
}

uint Ath__box::getWidth(uint* dir) const
{
  const uint dx = getDX();
  const uint dy = getDY();
  if (dx < dy) {
    *dir = 0;  // vertical
    return dx;
  }
  *dir = 1;  // horizontal
  return dy;
}

Ath__box::Ath__box(int x1, int y1, int x2, int y2, int units)
{
  set(x1, y1, x2, y2, units);
}

void Ath__box::set(int x1, int y1, int x2, int y2, int units)
{
  _rect = {x1 * units, y1 * units, x2 * units, y2 * units};
  _valid = 1;
  _id = 0;
}

uint Ath__box::getOwner() const
{
  return 0;
}

uint Ath__box::getDX() const
{
  return _rect.dx();
}

uint Ath__box::getDY() const
{
  return _rect.dy();
}

uint Ath__box::getLength() const
{
  return std::max(getDX(), getDY());
}

void Ath__box::invalidateBox()
{
  _valid = 0;
}

void Ath__box::set(Ath__box* bb)
{
  _rect = bb->_rect;
}

void Ath__searchBox::set(int x1, int y1, int x2, int y2, uint l, int dir)
{
  _ll[0] = x1;
  _ll[1] = y1;
  _ur[0] = x2;
  _ur[1] = y2;
  _level = l;
  setDir(dir);
  _ownId = 0;
  _otherId = 0;
  _type = 0;
}
Ath__searchBox::Ath__searchBox()
{
}
Ath__searchBox::Ath__searchBox(Ath__box* bb, uint l, int dir)
{
  const odb::Rect rect = bb->getRect();
  set(rect.xMin(), rect.yMin(), rect.xMax(), rect.yMax(), l, dir);
}
Ath__searchBox::Ath__searchBox(Ath__searchBox* bb, uint l, int dir)
{
  set(bb->_ll[0], bb->_ll[1], bb->_ur[0], bb->_ur[1], l, dir);
}
Ath__searchBox::Ath__searchBox(int x1, int y1, int x2, int y2, uint l, int dir)
{
  set(x1, y1, x2, y2, l, dir);
}
void Ath__searchBox::setMidPointSearch()
{
  for (uint i = 0; i < 2; i++) {
    _ll[i] = (_ll[i] + _ur[i]) / 2;
    _ur[i] = _ll[i] + 1;
  }
}

void Ath__searchBox::setLo(uint d, int xy)
{
  _ll[d] = xy;
}
void Ath__searchBox::setHi(uint d, int xy)
{
  _ur[d] = xy;
}
int Ath__searchBox::loXY(uint d)
{
  return _ll[d];
}
int Ath__searchBox::hiXY(uint d)
{
  return _ur[d];
}
int Ath__searchBox::loXY(uint d, int loBound)
{
  if (_ll[d] < loBound) {
    return loBound;
  }
  return _ll[d];
}
int Ath__searchBox::hiXY(uint d, int hiBound)
{
  if (_ur[d] > hiBound) {
    return hiBound;
  }
  return _ur[d];
}
uint Ath__searchBox::getDir()
{
  return _dir;
}
uint Ath__searchBox::getLevel()
{
  return _level;
}
uint Ath__searchBox::getOwnerId()
{
  return _ownId;
}
uint Ath__searchBox::getOtherId()
{
  return _otherId;
}
uint Ath__searchBox::getType()
{
  return _type;
}
void Ath__searchBox::setOwnerId(uint v, uint other)
{
  _ownId = v;
  _otherId = other;
}
void Ath__searchBox::setType(uint v)
{
  _type = v;
}
void Ath__searchBox::setDir(int dir)
{
  if (dir >= 0) {
    _dir = dir;
  } else {
    _dir = 1;  // horizontal
    int dx = _ur[0] - _ll[0];
    if (dx < _ur[1] - _ll[1]) {
      _dir = 0;  // vertical
    }
  }
}
uint Ath__searchBox::getLength()
{
  if (_dir > 0) {
    return _ur[0] - _ll[0];
  }
  return _ur[1] - _ll[1];
}
void Ath__searchBox::setLevel(uint v)
{
  _level = v;
}

void Ath__wire::reset()
{
  _boxId = 0;
  _srcId = 0;
  _otherId = 0;
  _track = NULL;
  _next = NULL;
  _upNext = NULL;
  _downNext = NULL;
  _aboveNext = NULL;
  _belowNext = NULL;
  _ouLen = 0;

  _xy = 0;  // offset from track start
  _len = 0;

  _base = 0;
  _width = 0;
  _flags = 0;  // ownership
               // 0=wire, 1=obs, 2=pin, 3=power
  _dir = 0;
  _ext = 0;
}
bool Ath__wire::isTilePin()
{
  if (_flags == 1) {
    return true;
  }
  return false;
}
bool Ath__wire::isTileBus()
{
  if (_flags == 2) {
    return true;
  }
  return false;
}
bool Ath__wire::isPower()
{
  uint power_wire_id = 11;  // see db/dbSearch.cpp
  if (_flags == power_wire_id) {
    return true;
  }
  return false;
}
bool Ath__wire::isVia()
{
  uint via_wire_id = 5;  // see db/dbSearch.cpp
  if (_flags == via_wire_id) {
    return true;
  }
  return false;
}
void Ath__wire::setOtherId(uint id)
{
  _otherId = id;
}
int Ath__wire::getRsegId()
{
  int wBoxId = _boxId;
  if (!(_otherId > 0)) {
    return wBoxId;
  }

  if (isVia()) {
    wBoxId = getShapeProperty(_otherId);
  } else {
    getNet()->getWire()->getProperty((int) _otherId, wBoxId);
  }
  return wBoxId;
}
int Ath__wire::getShapeProperty(int id)
{
  dbNet* net = getNet();
  if (net == nullptr) {
    return 0;
  }
  char buff[64];
  sprintf(buff, "%d", id);
  auto* p = odb::dbIntProperty::find(net, buff);
  if (p == nullptr) {
    return 0;
  }
  int rcid = p->getValue();
  return rcid;
}
dbNet* Ath__wire::getNet()
{
  Ath__gridTable* gtb = _track->getGrid()->getGridTable();
  dbBlock* block = gtb->getBlock();
  if (_otherId == 0) {
    return (odb::dbSBox::getSBox(block, _boxId)->getSWire()->getNet());
  }
  if (gtb->usingDbSdb()) {
    return dbNet::getNet(block, _boxId);
  }
  return (odb::dbRSeg::getRSeg(block, _boxId)->getNet());
}
uint Ath__wire::getBoxId()
{
  return _boxId;
}
uint Ath__wire::getOtherId()
{
  return _otherId;
}
uint Ath__wire::getSrcId()
{
  return _srcId;
}
void Ath__wire::set(uint dir, const int* ll, const int* ur)
{
  _boxId = 0;
  _srcId = 0;
  _track = nullptr;
  _next = nullptr;

  _dir = dir;
  uint d = (_dir > 0) ? 0 : 1;

  int xy1 = ll[d];
  int xy2 = ur[d];

  int yx1 = ll[dir];
  int yx2 = ur[dir];

  _xy = xy1;  // offset from track start
  _len = xy2 - xy1;

  _base = yx1;  // small dimension
  _width = yx2 - yx1;
  // OpenRCX
  _visited = 0;
  _ouLen = 0;
}
Ath__wire* Ath__track::getTargetWire()
{
  return _targetWire;
}
void Ath__track::initTargetWire(int noPowerWire)
{
  _targetWire = nullptr;
  for (_targetMarker = 0; _targetMarker < _markerCnt; _targetMarker++) {
    if (_marker[_targetMarker] == nullptr) {
      continue;
    }
    _targetWire = _marker[_targetMarker];
    while (_targetWire && noPowerWire && _targetWire->isPower()) {
      _targetWire = _targetWire->_next;
    }
    if (_targetWire) {
      break;
    }
  }
}
Ath__wire* Ath__track::nextTargetWire(int noPowerWire)
{
  if (_targetWire) {
    _targetWire = _targetWire->_next;
    while (_targetWire && noPowerWire && _targetWire->isPower()) {
      _targetWire = _targetWire->_next;
    }
    if (!_targetWire) {
      _targetMarker++;
    }
  }
  if (_targetWire) {
    return _targetWire;
  }
  for (; _targetMarker < _markerCnt; _targetMarker++) {
    if (_marker[_targetMarker] == nullptr) {
      continue;
    }
    _targetWire = _marker[_targetMarker];
    while (_targetWire && noPowerWire && _targetWire->isPower()) {
      _targetWire = _targetWire->_next;
    }
    if (_targetWire) {
      break;
    }
  }
  return _targetWire;
}

int Ath__wire::wireOverlap(Ath__wire* w, int* len1, int* len2, int* len3)
{
  // _xy, _len : reference rect

  int X1 = _xy;
  int DX = _len;
  int x1 = w->_xy;
  int dx = w->_len;

  //	fprintf(stdout, "%d %d   %d %d  : ", X1, DX,   x1, dx);

  int dx1 = X1 - x1;
  //*len1= dx1;
  if (dx1 >= 0)  // on left side
  {
    int dlen = dx - dx1;
    if (dlen <= 0) {
      return 1;
    }

    *len1 = 0;
    int DX2 = dlen - DX;

    if (DX2 <= 0) {
      *len2 = dlen;
      *len3 = -DX2;
    } else {
      *len2 = DX;
      //*len3= DX2;
      *len3 = 0;
    }
  } else {
    *len1 = -dx1;

    if (dx1 + DX <= 0) {  // outside right side
      return 2;
    }

    int DX2 = (x1 + dx) - (X1 + DX);
    if (DX2 > 0) {
      *len2 = DX + dx1;  // dx1 is negative
      *len3 = 0;
    } else {
      *len2 = dx;
      *len3 = -DX2;
    }
  }
  return 0;
}
void Ath__wire::getCoords(int* x1, int* y1, int* x2, int* y2, uint* dir)
{
  if (_dir > 0)  // horizontal
  {
    *x1 = _xy;
    *y1 = _base;
    *x2 = _xy + _len;
    *y2 = _base + _width;
  } else {
    *y1 = _xy;
    *x1 = _base;
    *y2 = _xy + _len;
    *x2 = _base + _width;
  }
  *dir = _dir;
}
void Ath__wire::getCoords(Ath__searchBox* box)
{
  uint level = _track->getGrid()->getLevel();
  if (_dir > 0)  // horizontal
  {
    box->set(_xy, _base, _xy + _len, _base + _width, level, _dir);
  } else {
    box->set(_base, _xy, _base + _width, _xy + _len, level, _dir);
  }
  box->setType(_flags);
}

Ath__track* Ath__track::getNextSubTrack(Ath__track* subt, bool tohi)
{
  if (!subt) {
    return tohi ? this : this->getLowTrack();
  }
  if (tohi) {
    return subt->getHiTrack()->_lowest ? nullptr : subt->getHiTrack();
  }
  return subt->_lowest ? nullptr : subt->getLowTrack();
}

void Ath__track::setHiTrack(Ath__track* hitrack)
{
  _hiTrack = hitrack;
}
void Ath__track::setLowTrack(Ath__track* lowtrack)
{
  _lowTrack = lowtrack;
}
Ath__track* Ath__track::getHiTrack()
{
  return _hiTrack;
}
Ath__track* Ath__track::getLowTrack()
{
  return _lowTrack;
}

void Ath__track::set(Ath__grid* g,
                     int x,
                     int y,
                     uint n,
                     uint width,
                     uint markerLen,
                     uint markerCnt,
                     int base)
{
  _grid = g;
  _x = x;
  _y = y;
  _num = n;
  _width = width;

  if (markerCnt <= 4) {
    _markerCnt = markerCnt;
    _marker = new Ath__wire*[4];
    _eMarker = new Ath__wire*[4];
  } else {
    _markerCnt = markerCnt;
    _marker = new Ath__wire*[_markerCnt];
    _eMarker = new Ath__wire*[_markerCnt];
  }
  for (uint ii = 0; ii < _markerCnt; ii++) {
    _marker[ii] = nullptr;
    _eMarker[ii] = nullptr;
  }

  _blocked = 1;
  _ordered = false;

  _hiTrack = this;
  _lowTrack = this;
  _lowest = 0;
  _base = base;
}
void Ath__track::freeWires(AthPool<Ath__wire>* pool)
{
  for (uint ii = 0; ii < _markerCnt; ii++) {
    Ath__wire* w = _marker[ii];
    while (w != nullptr) {
      Ath__wire* a = w->getNext();

      pool->free(w);
      w = a;
    }
  }
}
void Ath__track::dealloc(AthPool<Ath__wire>* pool)
{
  freeWires(pool);
  delete[] _marker;
  delete[] _eMarker;
}

uint Ath__grid::getAbsTrackNum(int xy)
{
  int dist = xy - _base;

  assert(dist >= 0);

  uint n = dist / _pitch;

  assert(n < _trackCnt);

  return n;
}
uint Ath__grid::getMinMaxTrackNum(int xy)
{
  int dist = xy - _base;

  if (dist < 0) {
    return 0;
  }

  uint n = dist / _pitch;

  if (n >= _trackCnt) {
    return _trackCnt - 1;
  }

  return n;
}

void Ath__grid::initContextTracks()
{
  setSearchDomain(1);
  Ath__track *track, *btrack;
  uint ii;
  bool noPowerTarget = _gridtable->noPowerTarget() > 0 ? true : false;
  for (ii = _searchLowTrack; ii <= _searchHiTrack; ii++) {
    btrack = _trackTable[ii];
    if (btrack == nullptr) {
      continue;
    }
    track = nullptr;
    bool tohi = true;
    while ((track = btrack->getNextSubTrack(track, tohi)) != nullptr) {
      track->initTargetWire(noPowerTarget);
    }
  }
}

void Ath__grid::initContextGrids()
{
  uint sdepth = _gridtable->contextDepth();
  if (sdepth == 0) {
    return;
  }
  uint ii = _dir ? 0 : 1;
  uint jj;
  for (jj = 1; jj <= sdepth && (jj + _level) < _gridtable->getColCnt(); jj++) {
    _gridtable->getGrid(ii, jj + _level)->initContextTracks();
  }
  for (jj = 1; jj <= sdepth && (_level - jj) > 0; jj++) {
    _gridtable->getGrid(ii, _level - jj)->initContextTracks();
  }
}

void Ath__grid::setSearchDomain(uint domainAdjust)
{
  if (_gridtable->allNet()) {
    _searchLowTrack = 0;
    _searchHiTrack = _trackCnt - 1;
    _searchLowMarker = 0;
    _searchHiMarker = _markerCnt - 1;
    return;
  }
  Ath__box* searchBox = _gridtable->maxSearchBox();
  const odb::Rect rect = searchBox->getRect();
  const odb::Orientation2D dir = _dir ? odb::horizontal : odb::vertical;
  const int lo = rect.low(dir.turn_90());
  const int hi = rect.high(dir.turn_90());
  const int ltrack = (int) getMinMaxTrackNum(lo) - (int) domainAdjust;
  _searchLowTrack = ltrack < 0 ? 0 : ltrack;
  _searchHiTrack = getMinMaxTrackNum(hi) + domainAdjust;
  if (_searchHiTrack >= _trackCnt) {
    _searchHiTrack = _trackCnt - 1;
  }
  const int mlo = rect.low(dir);
  const int mhi = rect.high(dir);
  _searchLowMarker = getBucketNum(mlo);
  _searchHiMarker = getBucketNum(mhi);
}

Ath__track* Ath__grid::addTrack(uint ii, uint markerCnt, int base)
{
  Ath__track* track = _trackPoolPtr->alloc();
  track->set(this, _start, _end, ii, _width, _markerLen, markerCnt, base);
  return track;
}
Ath__track* Ath__grid::addTrack(uint ii, uint markerCnt)
{
  int trackBase = _base + _pitch * ii;
  return addTrack(ii, markerCnt, trackBase);
}
Ath__track* Ath__grid::getTrackPtr(uint ii, uint markerCnt, int base)
{
  if (ii >= _trackCnt) {
    return nullptr;
  }

  if (_blockedTrackTable[ii] > 0) {
    return nullptr;
  }

  Ath__track* ntrack;
  Ath__track* ttrack = _trackTable[ii];
  while (ttrack) {
    if (ttrack->getBase() == base) {
      break;
    }
    ntrack = ttrack->getHiTrack();
    ttrack = ntrack == _trackTable[ii] ? nullptr : ntrack;
  }
  if (ttrack) {
    return ttrack;
  }
  ttrack = addTrack(ii, markerCnt, base);
  if (_trackTable[ii] == nullptr) {
    _trackTable[ii] = ttrack;
    ttrack->setLowest(1);
    return ttrack;
  }
  _subTrackCnt[ii]++;
  ntrack = _trackTable[ii];
  while (true) {
    if (ntrack->getBase() > base) {
      break;
    }
    ntrack = ntrack->getHiTrack();
    if (ntrack == _trackTable[ii]) {
      break;
    }
  }
  ntrack->getLowTrack()->setHiTrack(ttrack);
  ttrack->setHiTrack(ntrack);
  ttrack->setLowTrack(ntrack->getLowTrack());
  ntrack->setLowTrack(ttrack);
  if (base < _trackTable[ii]->getBase()) {
    _trackTable[ii]->setLowest(0);
    ttrack->setLowest(1);
    _trackTable[ii] = ttrack;
  }
  return ttrack;
}
Ath__track* Ath__grid::getTrackPtr(uint ii, uint markerCnt)
{
  int trackBase = _base + _pitch * ii;
  return getTrackPtr(ii, markerCnt, trackBase);
}
bool Ath__track::place(Ath__wire* w, int markIndex1, int markIndex2)
{
  assert(markIndex1 >= 0);
  assert(markIndex2 >= 0);

  for (int ii = markIndex1 + 1; ii <= markIndex2; ii++) {
    _marker[ii] = w;
  }

  if (_marker[markIndex1] == nullptr) {
    _marker[markIndex1] = w;
    return true;
  }

  Ath__wire* a = _marker[markIndex1];
  if (w->_xy < a->_xy) {
    if (w->_xy + w->_len >= a->_xy) {
      fprintf(stdout, "overlap %d %d \n", w->_xy, a->_xy);
      return false;
    }
    w->setNext(a);
    _marker[markIndex1] = w;
    return true;
  }

  Ath__wire* e = _marker[markIndex1];
  for (; e != nullptr; e = e->_next) {
    if (w->_xy < e->_xy) {
      continue;
    }
    if (w->_xy + w->_len >= a->_xy) {
      fprintf(stdout, "overlap %d %d \n", w->_xy, a->_xy);
      return false;
    }
    w->setNext(e);
    break;
  }
  return false;
}
void Ath__wire::search(int xy1, int xy2, uint& cnt, Ath__array1D<uint>* idTable)
{
  Ath__wire* e = this;
  for (; e != nullptr; e = e->_next) {
    if (xy2 <= e->_xy) {
      break;
    }

    if ((xy1 <= e->_xy) && (xy2 >= e->_xy)) {
      idTable->add(e->_boxId);
      cnt++;
    } else if ((e->_xy <= xy1) && (e->_xy + e->_len >= xy1)) {
      idTable->add(e->_boxId);
      cnt++;
    }
  }
}
void Ath__wire::search1(int xy1,
                        int xy2,
                        uint& cnt,
                        Ath__array1D<uint>* idTable)
{
  Ath__wire* e = this;
  for (; e != nullptr; e = e->_next) {
    if (xy2 <= e->_xy) {
      break;
    }

    if ((xy1 <= e->_xy) && (xy2 >= e->_xy)) {
      idTable->add(e->_id);
      cnt++;
    } else if ((e->_xy <= xy1) && (e->_xy + e->_len >= xy1)) {
      idTable->add(e->_id);
      cnt++;
    }
  }
}
uint Ath__track::search(int xy1,
                        int xy2,
                        uint markIndex1,
                        uint markIndex2,
                        Ath__array1D<uint>* idTable)
{
  uint cnt = 0;
  if (_eMarker[markIndex1]) {
    _eMarker[markIndex1]->search(xy1, xy2, cnt, idTable);
  }
  for (uint ii = markIndex1; ii <= markIndex2; ii++) {
    if (_marker[ii] == nullptr) {
      continue;
    }
    _marker[ii]->search(xy1, xy2, cnt, idTable);
  }
  return cnt;
}
void Ath__track::resetExtFlag(uint markerCnt)
{
  for (uint ii = 0; ii < markerCnt; ii++) {
    Ath__wire* e = _marker[ii];
    for (; e != nullptr; e = e->_next) {
      e->_ext = 0;
    }
  }
}
uint Ath__track::getAllWires(Ath__array1D<Ath__wire*>* boxTable, uint markerCnt)
{
  for (uint ii = 0; ii < markerCnt; ii++) {
    Ath__wire* e = _marker[ii];
    for (; e != nullptr; e = e->_next) {
      if (e->_ext > 0) {
        continue;
      }

      e->_ext = 1;
      boxTable->add(e);
    }
  }
  resetExtFlag(markerCnt);
  return boxTable->getCnt();
}
uint Ath__track::search1(int xy1,
                         int xy2,
                         uint markIndex1,
                         uint markIndex2,
                         Ath__array1D<uint>* idTable)
{
  if (!_ordered) {
    markIndex1 = 0;
  }

  uint cnt = 0;
  if (_eMarker[markIndex1]) {
    _eMarker[markIndex1]->search1(xy1, xy2, cnt, idTable);
  }
  for (uint ii = markIndex1; ii <= markIndex2; ii++) {
    if (_marker[ii] == nullptr) {
      continue;
    }
    _marker[ii]->search1(xy1, xy2, cnt, idTable);
  }
  return cnt;
}
uint Ath__track::setExtrusionMarker(int markerCnt, int start, uint markerLen)
{
  _ordered = true;

  int jj;
  int cnt = 0;
  int ii;
  for (ii = 0; ii < markerCnt; ii++) {
    _eMarker[ii] = nullptr;
  }
  for (ii = 0; ii < markerCnt - 1; ii++) {
    for (Ath__wire* e = _marker[ii]; e != nullptr; e = e->_next) {
      int tailMark = (e->_xy + e->_len - start) / markerLen;
      if (tailMark == ii) {
        continue;
      }
      if (tailMark > markerCnt - 1) {
        tailMark = markerCnt - 1;
      }
      for (jj = ii + 1; jj <= tailMark; jj++) {
        _eMarker[jj] = e;
        if (_marker[jj]) {
          jj++;
          break;
        }
      }
      ii = jj - 2;
      cnt++;
      break;
    }
  }
  return cnt;
}
bool Ath__track::placeTrail(Ath__wire* w, uint m1, uint m2)
{
  for (uint ii = m1 + 1; ii <= m2; ii++) {
    if (_marker[ii] == nullptr) {
      _marker[ii] = w;
      continue;
    }
    if (w->_xy <= _marker[ii]->_xy) {
      w->setNext(_marker[ii]);
      _marker[ii] = w;
    } else {
      w->setNext(_marker[ii]->_next);
      _marker[ii]->setNext(w);
    }
  }
  return true;
}
bool Ath__track::checkAndplacerOnMarker(Ath__wire* w, int markIndex)
{
  if (_marker[markIndex] == nullptr) {
    _marker[markIndex] = w;
    return true;
  }
  return false;
}
bool Ath__track::checkMarker(int markIndex)
{
  if (_marker[markIndex] == nullptr) {
    return true;
  }
  return false;
}
bool Ath__track::checkAndplace(Ath__wire* w, int markIndex1)
{
  if (_marker[markIndex1] == nullptr) {
    _marker[markIndex1] = w;
    return true;
  }

  Ath__wire* a = _marker[markIndex1];
  if (w->_xy <= a->_xy) {
    if (w->_xy + w->_len > a->_xy) {
      return false;
    }

    w->setNext(a);
    _marker[markIndex1] = w;

    return true;
  }
  Ath__wire* prev = _marker[markIndex1];
  Ath__wire* e = _marker[markIndex1];
  for (; e != nullptr; e = e->_next) {
    if (w->_xy <= e->_xy) {
      if (w->_xy + w->_len > e->_xy) {
        return false;
      }

      w->setNext(e);
      prev->setNext(w);
      return true;
    }
    prev = e;
  }

  if (prev->_xy + prev->_len > w->_xy) {
    return false;
  }
  prev->setNext(w);
  return true;
}
void Ath__track::insertWire(Ath__wire* w, int mark1, int mark2)
{
  w->_track = this;
  for (int ii = mark1; ii < mark2; ii++) {
    _marker[ii] = w;
  }
  if (mark2 > mark1) {
    w->setNext(_marker[mark2]);
    _marker[mark2] = w;
  }
}

bool Ath__track::place2(Ath__wire* w, int mark1, int mark2)
{
  assert(mark1 >= 0);

  w->_next = nullptr;
  if (_marker[mark1] == nullptr) {
    insertWire(w, mark1, mark2);
    return true;
  }
  bool status = true;

  Ath__wire* a = _marker[mark1];
  if (w->_xy <= a->_xy) {
    w->setNext(a);
    _marker[mark1] = w;

    w->_track = this;

    return true;
  }
  Ath__wire* prev = _marker[mark1];
  Ath__wire* e = _marker[mark1];
  for (; e != nullptr; e = e->_next) {
    if (w->_xy <= e->_xy) {
      w->setNext(e);
      prev->setNext(w);

      w->_track = this;
      return true;
    }
    prev = e;
  }
  if (e == nullptr) {  // at the end of the list
    prev->setNext(w);
    insertWire(w, mark1, mark2);
    return true;
  }

  if (!status) {
    fprintf(stdout, "OVERLAP placement\n");
  }

  return status;
}

void Ath__track::linkWire(Ath__wire*& w1, Ath__wire*& w2)
{
  Ath__overlapAdjust adj
      = (Ath__overlapAdjust) _grid->getGridTable()->getOverlapAdjust();
  int nend, oend, tend;
  nend = w1->_xy + w1->_len;
  oend = w2->_xy + w2->_len;
  tend = nend > oend ? nend : oend;
  if (adj == Z_noAdjust || nend <= w2->_xy) {
    w1->setNext(w2);
  } else if (w1->_base != w2->_base || w1->_width != w2->_width) {
    if (!_grid->getGridTable()->getOverlapTouchCheck()
        || w1->_base > w2->_base + w2->_width
        || w2->_base > w1->_base + w1->_width) {
      w1->setNext(w2);
    } else {                        // only good for adj == Z_endAdjust
      if (w1->_base < w2->_base) {  // w1 is wider?
        w2->_xy = nend;
        w1->setNext(w2);
        if (nend >= oend) {
          w2->_len = 0;
        } else {
          w2->_len = oend - nend;
        }
      } else {  // todo: nend > oend
        w1->setNext(w2);
        w1->_len = w2->_xy - w1->_xy;
      }
    }
  } else if (adj == Z_merge) {
    w1->_len = tend - w1->_xy;
    w1->setNext(w2->_next);
    w2 = w1;
  } else {  // adj == Z_endAdjust
    w2->_xy = nend;
    w1->setNext(w2);
    if (nend >= oend) {
      w2->_len = 0;
    } else {
      w2->_len = oend - nend;
    }
  }
}

bool Ath__track::place(Ath__wire* w, int markIndex1)
{
  assert(markIndex1 >= 0);

  w->_track = this;

  if (_marker[markIndex1] == nullptr) {
    _marker[markIndex1] = w;
    return true;
  }
  Ath__overlapAdjust adj
      = (Ath__overlapAdjust) _grid->getGridTable()->getOverlapAdjust();
  bool status = true;

  Ath__wire* a = _marker[markIndex1];
  if (w->_xy <= a->_xy) {
    if (adj != Z_noAdjust && w->_xy + w->_len > a->_xy + a->_len
        && w->_base <= a->_base
        && w->_base + w->_width >= a->_base + a->_width) {  // a inside w
      w->_next = _marker[markIndex1]->_next;
      _marker[markIndex1] = w;
      return true;
    }
    linkWire(w, a);
    _marker[markIndex1] = w;

    return true;
  }
  Ath__wire* prev = _marker[markIndex1];
  Ath__wire* e = _marker[markIndex1];
  for (; e != nullptr; e = e->_next) {
    if (w->_xy <= e->_xy) {
      if (adj != Z_noAdjust && w->_xy + w->_len >= e->_xy + e->_len
          && w->_base <= e->_base
          && w->_base + w->_width >= e->_base + e->_width) {  // e inside w
        prev->_next = w;
        w->_next = e->_next;
        return true;
      }
      linkWire(prev, w);
      linkWire(w, e);
      return true;
    }
    if (adj != Z_noAdjust && w->_xy + w->_len <= e->_xy + e->_len
        && e->_base <= w->_base
        && e->_base + e->_width >= w->_base + w->_width) {  // w inside e
      return true;
    }
    prev = e;
  }
  if (e == nullptr) {
    linkWire(prev, w);
  }

  if (!status) {
    fprintf(stdout, "OVERLAP placement\n");
  }

  return status;
}
Ath__wire* Ath__track::getNextWire(Ath__wire* wire)
{
  Ath__wire* nwire;
  if (!wire) {
    _searchMarkerIndex = _grid->searchLowMarker();
    nwire = _marker[_searchMarkerIndex];
  } else {
    nwire = wire->_next;
  }
  if (nwire) {
    return nwire;
  }
  for (_searchMarkerIndex++; _searchMarkerIndex <= _grid->searchHiMarker();
       _searchMarkerIndex++) {
    nwire = _marker[_searchMarkerIndex];
    if (nwire) {
      return nwire;
    }
  }
  return nullptr;
}

Ath__wire* Ath__track::getWire_Linear(uint markerCnt, uint id)
{
  for (uint ii = 0; ii < markerCnt; ii++) {
    Ath__wire* e = _marker[ii];
    for (; e != nullptr; e = e->_next) {
      if (e->_id == id) {
        return e;
      }
    }
  }
  return nullptr;
}
void Ath__track::adjustOverlapMakerEnd(uint markerCnt,
                                       int start,
                                       uint markerLen)
{
  _ordered = true;

  Ath__wire* e;
  uint tailMark;
  uint jj;
  for (uint ii = 0; ii < markerCnt - 1; ii++) {
    e = _marker[ii];
    if (e == nullptr) {
      continue;
    }
    for (; e->_next != nullptr; e = e->_next) {
      ;
    }
    tailMark = (e->_xy + e->_len - start) / markerLen;
    if (tailMark == ii) {
      continue;
    }
    if (tailMark > markerCnt - 1) {
      tailMark = markerCnt - 1;
    }
    for (jj = ii + 1; jj <= tailMark; jj++) {
      _eMarker[jj] = e;
      if (_marker[jj]) {
        jj++;
        break;
      }
    }
    jj--;
    if (_marker[jj] != nullptr && e->_xy + e->_len > _marker[jj]->_xy) {
      e->_len = _marker[jj]->_xy - e->_xy;
    }
    ii = jj - 1;
  }
}
void Ath__track::adjustOverlapMakerEnd(uint markerCnt)
{
  Ath__wire* e;
  uint jj;
  for (uint ii = 0; ii < markerCnt - 1; ii++) {
    e = _marker[ii];
    if (e == nullptr) {
      continue;
    }
    for (; e->_next != nullptr; e = e->_next) {
      ;
    }
    for (jj = ii + 1; jj < markerCnt && _marker[jj] == nullptr; jj++) {
      ;
    }
    if (jj == markerCnt) {
      continue;
    }
    if (e->_xy + e->_len > _marker[jj]->_xy) {
      e->_len = _marker[jj]->_xy - e->_xy;
    }
    ii = jj - 1;
  }
}

bool Ath__track::isAscendingOrdered(uint markerCnt, uint* wCnt)
{
  uint cnt = 0;
  for (uint ii = 0; ii < markerCnt; ii++) {
    Ath__wire* e = _marker[ii];
    for (; e != nullptr; e = e->_next) {
      Ath__wire* w = e->_next;
      cnt++;

      if (w == nullptr) {
        break;
      }

      if (w->_xy < e->_xy) {
        *wCnt = cnt;
        return false;
      }
    }
  }
  *wCnt += cnt;
  return true;
}
bool Ath__track::overlapCheck(Ath__wire* w,
                              int markIndex1,
                              int /* unused: markIndex2 */)
{
  assert(markIndex1 >= 0);

  Ath__wire* e = _marker[markIndex1];
  for (; e != nullptr; e = e->_next) {
    if (w->_xy + w->_len >= e->_xy) {
      return true;
    }
  }
  return false;
}
void Ath__grid::makeTrackTable(uint width, uint pitch, uint space)
{
  if (width > 0) {
    _width = width;
  }

  if (space > 0) {
    _pitch = _width + space;
  } else if (pitch > 0) {
    _pitch = pitch;
  }

  _markerLen = ((_end - _start) / _width) / _markerCnt;

  _trackCnt = _max;  // for LINUX assert
  _trackCnt = getAbsTrackNum(_max) + 1;
  _subTrackCnt = (uint*) calloc(sizeof(uint), _trackCnt);
  _trackTable = new Ath__track*[_trackCnt];
  _blockedTrackTable = new uint[_trackCnt];

  for (uint ii = 0; ii < _trackCnt; ii++) {
    _trackTable[ii] = nullptr;
    _blockedTrackTable[ii] = 0;
  }
}

void Ath__grid::setBoundaries(uint dir, const odb::Rect& rect)
{
  _lo[0] = rect.xMin();
  _lo[1] = rect.yMin();
  _hi[0] = rect.xMax();
  _hi[1] = rect.yMax();

  _dir = dir;
  if (_dir == 0) {  // vertical
    _base = rect.xMin();
    _max = rect.xMax();
    _start = rect.yMin();
    _end = rect.yMax();
  } else {
    _base = rect.yMin();
    _max = rect.yMax();
    _start = rect.xMin();
    _end = rect.xMax();
  }
}
Ath__grid::Ath__grid(Ath__gridTable* gt,
                     AthPool<Ath__track>* trackPool,
                     AthPool<Ath__wire>* wirePool,
                     uint level,
                     uint num,
                     uint markerCnt)
{
  _gridtable = gt;
  _trackPoolPtr = trackPool;
  _wirePoolPtr = wirePool;
  _markerCnt = markerCnt;
  _level = level;
  _layer = num;
  _schema = 0;
}

void Ath__grid::setTracks(uint dir,
                          uint width,
                          uint pitch,
                          int xlo,
                          int ylo,
                          int xhi,
                          int yhi,
                          uint markerLen)
{
  setBoundaries(dir, {xlo, ylo, xhi, yhi});
  makeTrackTable(width, pitch);

  if (markerLen > 0) {
    _markerLen = markerLen;
    _markerCnt = ((_end - _start) / _width) / markerLen;
    if (_markerCnt == 0) {
      _markerCnt = 1;
    }
  }
}
void Ath__grid::setSchema(uint v)
{
  _schema = v;
}
Ath__grid::Ath__grid(Ath__gridTable* gt,
                     AthPool<Ath__track>* trackPool,
                     AthPool<Ath__wire>* wirePool,
                     Ath__box* bb,
                     uint level,
                     uint dir,
                     uint num,
                     uint width,
                     uint pitch,
                     uint markerCnt)
{
  _gridtable = gt;
  _trackPoolPtr = trackPool;
  _wirePoolPtr = wirePool;
  _markerCnt = markerCnt;

  _level = level;
  _layer = num;

  setBoundaries(dir, bb->getRect());
  makeTrackTable(width, pitch);
  _schema = 0;
}
void Ath__grid::getBbox(Ath__searchBox* bb)
{
  if (_dir == 0) {  // vertical
    bb->set(_base, _start, _max, _end, _level, _dir);
  } else {
    bb->set(_start, _base, _end, _max, _level, _dir);
  }
}
void Ath__grid::getBbox(Ath__box* bb)
{
  if (_dir == 0) {  // vertical
    bb->setRect({_base, _start, _max, _end});
  } else {
    bb->setRect({_start, _base, _end, _max});
  }
}

void Ath__grid::freeTracksAndTables()
{
  delete[] _trackTable;
  delete[] _blockedTrackTable;
}
Ath__grid::~Ath__grid()
{
  freeTracksAndTables();
}
uint Ath__grid::getLevel()
{
  return _level;
}
uint Ath__grid::getDir()
{
  return _dir;
}
int Ath__grid::getTrackHeight(uint track)
{
  return _base + track * _pitch;
}
Ath__grid* Ath__track::getGrid()
{
  return _grid;
}
uint Ath__track::removeMarkedNetWires()
{
  uint cnt = 0;
  Ath__wire *wire, *pwire, *nwire;
  for (uint jj = 0; jj < _markerCnt; jj++) {
    pwire = nullptr;
    wire = _marker[jj];
    _marker[jj] = nullptr;
    for (; wire != nullptr; wire = nwire) {
      nwire = wire->_next;
      if (wire->isPower() || !wire->getNet()->isMarked()) {
        if (wire->isPower() || wire->getNet()->getWire() != nullptr) {
          pwire = wire;
          if (_marker[jj] == nullptr) {
            _marker[jj] = wire;
          }
          continue;
        }
      }
      if (pwire) {
        pwire->_next = nwire;
      }
      for (uint kk = jj + 1; kk < _markerCnt; kk++) {
        if (_eMarker[kk] == wire) {
          _eMarker[kk] = nwire;
        }
      }
      _grid->getWirePoolPtr()->free(wire);
      cnt++;
    }
  }
  return cnt;
}
uint Ath__grid::defaultWireType()
{
  return _wireType;
}
void Ath__grid::setDefaultWireType(uint v)
{
  _wireType = v;  // TODO-OPTIMIZE : can it be 32-bit?
}
uint Ath__grid::getBoxes(uint trackIndex, Ath__array1D<uint>* table)
{
  Ath__track* tr = _trackTable[trackIndex];
  if (tr == nullptr) {
    return 0;
  }
  if (_blockedTrackTable[trackIndex] > 0) {
    return 0;
  }

  for (uint k = 0; k < tr->_markerCnt; k++) {
    if (tr->_marker[k] == nullptr) {
      continue;
    }
    for (Ath__wire* w = tr->_marker[k]; w != nullptr; w = w->_next) {
      table->add(w->_boxId);
    }
  }
  return table->getCnt();
}
void Ath__grid::getBoxes(Ath__array1D<uint>* table)
{
  for (uint ii = 0; ii < _trackCnt; ii++) {
    Ath__track* tr = _trackTable[ii];
    if (tr == nullptr) {
      continue;
    }
    if (_blockedTrackTable[ii] > 0) {
      continue;
    }

    for (Ath__wire* w = tr->_marker[0]; w != nullptr; w = w->_next) {
      table->add(w->_boxId);
    }
  }
}

bool Ath__grid::addOnTrack(uint track, Ath__wire* w, uint mark1, uint mark2)
{
  if (_blockedTrackTable[track] > 0) {
    return false;
  }

  Ath__track* tr = nullptr;
  if (_trackTable[track] == nullptr) {
    tr = addTrack(track, _markerCnt);
    _trackTable[track] = tr;
    if (tr->place(w, mark1, mark2)) {
      w->_track = tr;
      return true;
    }
    return false;
  }
  if (!_trackTable[track]->overlapCheck(w, mark1, mark2)) {
    tr = _trackTable[track];
    if (tr->place(w, mark1, mark2)) {
      w->_track = tr;
      return true;
    }
    return false;
  }
  return false;
}
uint Ath__grid::placeWire(uint initTrack,
                          Ath__wire* w,
                          uint mark1,
                          uint mark2,
                          int sortedOrder,
                          int* height)
{
  uint check = 20;
  uint track = initTrack;

  uint nextTrack = track;

  bool status = false;
  if (sortedOrder > 0) {
    for (; (track < _trackCnt) && (track < initTrack + check); track++) {
      status = addOnTrack(track, w, mark1, mark2);
      if (status) {
        break;
      }
    }
    nextTrack = track + 1;
  } else {
    for (; (!status) && (track > 0) && (track > initTrack - check); track--) {
      status = addOnTrack(track, w, mark1, mark2);
      if (status) {
        break;
      }
    }
    nextTrack = track - 1;
  }
  if (!status) {
    fprintf(stdout, "Cannot place at track# %d\n", initTrack);
    *height = getTrackHeight(initTrack);
    return initTrack;
  }
  *height = getTrackHeight(track);
  return nextTrack;
}

uint Ath__grid::addWire(uint initTrack,
                        Ath__box* box,
                        int sortedOrder,
                        int* height)
{
  uint id, markIndex1, markIndex2;
  Ath__wire* w = makeWire(box, &id, &markIndex1, &markIndex2, 0);

  return placeWire(initTrack, w, markIndex1, markIndex2, sortedOrder, height);
}
Ath__track* Ath__grid::getTrackPtr(int xy)
{
  uint trackNum = getMinMaxTrackNum(xy);

  return getTrackPtr(trackNum, _markerCnt);
}
Ath__track* Ath__grid::getTrackPtr(int* ll)
{
  uint trackNum = getMinMaxTrackNum(ll[_dir]);

  return getTrackPtr(trackNum, _markerCnt);
}
uint Ath__grid::placeBox(uint id, int x1, int y1, int x2, int y2)
{
  int ll[2] = {x1, y1};
  int ur[2] = {x2, y2};

  uint d = (_dir > 0) ? 0 : 1;

  int xy1 = ll[d];

  uint m1 = getBucketNum(xy1);
  int width = ur[_dir] - ll[_dir];

  uint trackNum1 = getMinMaxTrackNum(ll[_dir]);
  uint trackNum2 = trackNum1;
  if (width > _pitch) {
    trackNum2 = getMinMaxTrackNum(ur[_dir]);
  }

  for (uint ii = trackNum1; ii <= trackNum2; ii++) {
    Ath__wire* w = makeWire(_dir, ll, ur, id, 0);

    Ath__track* track = getTrackPtr(ii, _markerCnt);

    if (track->place(w, m1)) {
      w->_track = track;
    } else {
      fprintf(stdout, "OVERLAP placement\n");
    }
  }

  return trackNum1;
}
void Ath__wire::setXY(int xy1, uint len)
{
  _xy = xy1;  // offset from track start??
  _len = len;
}
Ath__wire* Ath__wire::makeCoupleWire(AthPool<Ath__wire>* wirePool,
                                     int targetHighTracks,
                                     Ath__wire* w2,
                                     int xy1,
                                     uint len,
                                     uint /* unused: wtype */)
{
  int dist;
  if (targetHighTracks) {
    dist = w2->_base - (_base + _width);
  } else {
    dist = _base - (w2->_base + w2->_width);
  }
  if (dist <= 0) {
    return nullptr;
  }

  Ath__wire* w = getPoolWire(wirePool);
  w->_srcId = 0;

  w->reset();

  w->_xy = xy1;  // offset from track start??
  w->_len = len;

  w->_width = dist;
  w->_boxId = _id;
  w->_otherId = w2->_id;
  w->_flags = _flags;
  w->_dir = _dir;
  if (targetHighTracks) {
    w->_base = _base + _width;  // small dimension
  } else {
    w->_base = w2->_base + w2->_width;
  }
  return w;
}
Ath__wire* Ath__wire::getPoolWire(AthPool<Ath__wire>* wirePool)
{
  int n;
  int getRecycleFlag = 0;
  Ath__wire* w = wirePool->alloc(&getRecycleFlag, &n);
  if (getRecycleFlag == 0) {
    w->_id = n;
  }
  return w;
}
Ath__wire* Ath__wire::makeWire(AthPool<Ath__wire>* wirePool, int xy1, uint len)
{
  Ath__wire* w = getPoolWire(wirePool);

  w->_srcId = 0;

  w->reset();

  w->_xy = xy1;  // offset from track start
  w->_len = len;

  w->_base = _base;  // small dimension
  w->_width = _width;

  w->_boxId = _boxId;
  w->_otherId = _otherId;

  w->_flags = _flags;
  w->_dir = _dir;

  return w;
}
Ath__wire* Ath__grid::getPoolWire()
{
  int n;
  int getRecycleFlag = 0;
  Ath__wire* w = _wirePoolPtr->alloc(&getRecycleFlag, &n);
  if (getRecycleFlag == 0) {
    w->_id = n;
  }
  return w;
}
Ath__wire* Ath__grid::makeWire(Ath__wire* v, uint type)
{
  Ath__wire* w = getPoolWire();
  w->_srcId = 0;

  w->reset();

  w->_xy = v->_xy;  // offset from track start
  w->_len = v->_len;

  w->_base = v->_base;  // small dimension
  w->_width = v->_width;

  w->_boxId = v->_boxId;
  w->_otherId = v->_otherId;

  w->_flags = type;

  w->_dir = v->_dir;

  return w;
}

uint Ath__grid::placeWire(Ath__searchBox* bb)
{
  uint d = !_dir;

  int xy1 = bb->loXY(d);

  int ll[2] = {bb->loXY(0), bb->loXY(1)};
  int ur[2] = {bb->hiXY(0), bb->hiXY(1)};

  uint m1 = getBucketNum(xy1);

  uint trackNum1 = getMinMaxTrackNum(bb->loXY(_dir));
  uint trackNum2 = getMinMaxTrackNum(bb->hiXY(_dir));

  uint wireType = bb->getType();

  Ath__wire* w
      = makeWire(_dir, ll, ur, bb->getOwnerId(), bb->getOtherId(), wireType);
  Ath__track* track;
  int TTTsubt = 1;
  if (TTTsubt) {
    track = getTrackPtr(trackNum1, _markerCnt, w->_base);
  } else {
    track = getTrackPtr(trackNum1, _markerCnt);
  }
  // track->place2(w, m1, m2);
  track->place(w, m1);
  for (uint ii = trackNum1 + 1; ii <= trackNum2; ii++) {
    Ath__wire* w1 = makeWire(w, wireType);
    w1->_srcId = w->_id;
    w1->_srcWire = w;
    _gridtable->incrMultiTrackWireCnt(w->isPower());
    Ath__track* track = getTrackPtr(ii, _markerCnt);
    track->place(w1, m1);
  }

  return trackNum1;
}
uint Ath__grid::placeWire(Ath__wire* w)
{
  uint m1 = getBucketNum(w->_xy);

  uint trackNum1 = getMinMaxTrackNum(w->_base);
  uint trackNum2 = getMinMaxTrackNum(w->_base + w->_width);

  Ath__track* track = getTrackPtr(trackNum1, _markerCnt);
  track->place(w, m1);

  for (uint ii = trackNum1 + 1; ii <= trackNum2; ii++) {
    Ath__wire* w1 = makeWire(w, w->_flags);
    w1->_srcId = w->_id;
    _gridtable->incrMultiTrackWireCnt(w->isPower());
    Ath__track* track = getTrackPtr(ii, _markerCnt);
    track->place(w1, m1);
  }

  return trackNum1;
}
uint Ath__grid::placeBox(dbBox* box, uint wtype, uint id)
{
  int ll[2] = {box->xMin(), box->yMin()};
  int ur[2] = {box->xMax(), box->yMax()};

  uint d = (_dir > 0) ? 0 : 1;

  int xy1 = ll[d];

  uint m1 = getBucketNum(xy1);

  int width = ur[_dir] - ll[_dir];

  uint trackNum1 = getMinMaxTrackNum(ll[_dir]);
  uint trackNum2 = trackNum1;
  if (width > _pitch) {
    trackNum2 = getMinMaxTrackNum(ur[_dir]);
  }

  if (id == 0) {
    id = box->getId();
  }
  Ath__wire* w = makeWire(_dir, ll, ur, id, 0, wtype);
  Ath__track* track = getTrackPtr(trackNum1, _markerCnt);
  track->place(w, m1);

  for (uint ii = trackNum1 + 1; ii <= trackNum2; ii++) {
    Ath__wire* w1 = makeWire(w);
    w1->_srcId = w->_id;
    _gridtable->incrMultiTrackWireCnt(w->isPower());
    Ath__track* track = getTrackPtr(ii, _markerCnt);
    track->place(w1, m1);
  }
  return trackNum1;
}
uint Ath__grid::setExtrusionMarker()
{
  Ath__track *track, *tstr;
  uint cnt = 0;
  for (uint ii = 0; ii < _trackCnt; ii++) {
    track = _trackTable[ii];
    if (track == nullptr) {
      continue;
    }
    tstr = nullptr;
    bool tohi = true;
    while ((tstr = track->getNextSubTrack(tstr, tohi)) != nullptr) {
      cnt += tstr->setExtrusionMarker(_markerCnt, _start, _markerLen);
    }
  }
  return cnt;
}
uint Ath__grid::placeBox(Ath__box* box)
{
  const odb::Rect rect = box->getRect();
  int ll[2] = {rect.xMin(), rect.yMin()};
  int ur[2] = {rect.xMax(), rect.yMax()};

  uint markIndex1;
  Ath__wire* w = makeWire(ll, ur, box->getOwner(), &markIndex1);

  Ath__track* track = getTrackPtr(ll);

  if (!track->place(w, markIndex1)) {
    fprintf(stdout, "OVERLAP placement\n");
  } else {
    w->_track = track;
  }

  return track->_num;
}
Ath__wire* Ath__grid::getWirePtr(uint wireId)
{
  return _wirePoolPtr->get(wireId);
}
void Ath__grid::getBoxIds(Ath__array1D<uint>* wireIdTable,
                          Ath__array1D<uint>* idtable)
{
  // remove duplicate entries

  for (uint ii = 0; ii < wireIdTable->getCnt(); ii++) {
    uint wid = wireIdTable->get(ii);
    Ath__wire* w = getWirePtr(wid);

    uint boxId = w->_boxId;
    if (w->_srcId > 0) {
      w = getWirePtr(w->_srcId);
      boxId = w->_boxId;
    }
    if (w->_ext > 0) {
      continue;
    }

    w->_ext = 1;
    idtable->add(boxId);
  }

  for (uint jj = 0; jj < wireIdTable->getCnt(); jj++) {
    Ath__wire* w = getWirePtr(wireIdTable->get(jj));
    w->_ext = 0;

    if (w->_srcId > 0) {
      w = getWirePtr(w->_srcId);
      w->_ext = 0;
    }
  }
}
void Ath__grid::getWireIds(Ath__array1D<uint>* wireIdTable,
                           Ath__array1D<uint>* idtable)
{
  // remove duplicate entries

  for (uint ii = 0; ii < wireIdTable->getCnt(); ii++) {
    uint wid = wireIdTable->get(ii);
    Ath__wire* w = getWirePtr(wid);

    if (w->_srcId > 0) {
      w = getWirePtr(w->_srcId);
      wid = w->_id;
    }
    if (w->_ext > 0) {
      continue;
    }

    w->_ext = 1;
    idtable->add(wid);
  }

  for (uint jj = 0; jj < wireIdTable->getCnt(); jj++) {
    Ath__wire* w = getWirePtr(wireIdTable->get(jj));
    w->_ext = 0;
    if (w->_srcId > 0) {
      w = getWirePtr(w->_srcId);
      w->_ext = 0;
    }
  }
}
uint Ath__grid::search(Ath__searchBox* bb,
                       Ath__array1D<uint>* idtable,
                       bool wireIdFlag)
{
  Ath__array1D<uint> wireIdTable(16000);

  // uint d= (_dir>0) ? 0 : 1;
  uint d = !_dir;

  uint loTrackNum = getTrackNum1(bb->loXY(_dir));
  if (loTrackNum > 0) {
    loTrackNum--;
  }

  uint hiTrackNum = getTrackNum1(bb->hiXY(_dir));

  int loXY = bb->loXY(d);
  int hiXY = bb->hiXY(d);
  uint loMarker = getBucketNum(loXY);
  uint hiMarker = getBucketNum(hiXY);

  Ath__track* tstrack;
  for (uint ii = loTrackNum; ii <= hiTrackNum; ii++) {
    Ath__track* track = _trackTable[ii];
    if (track == nullptr) {
      continue;
    }

    tstrack = nullptr;
    bool tohi = true;
    while ((tstrack = track->getNextSubTrack(tstrack, tohi)) != nullptr) {
      if (_schema > 0) {
        tstrack->search1(loXY, hiXY, loMarker, hiMarker, &wireIdTable);
      } else {
        tstrack->search(loXY, hiXY, loMarker, hiMarker, idtable);
      }
    }
  }
  if (wireIdFlag) {
    getWireIds(&wireIdTable, idtable);
  } else {
    getBoxIds(&wireIdTable, idtable);
  }

  return idtable->getCnt();
}
uint Ath__grid::search(Ath__searchBox* bb,
                       const uint* gxy,
                       Ath__array1D<uint>* idtable,
                       Ath__grid* g)
{
  Ath__array1D<uint> wireIdTable(1024);

  AthPool<Ath__wire>* wirePool = _wirePoolPtr;
  if (g != nullptr) {
    wirePool = g->getWirePoolPtr();
  }

  uint d = !_dir;

  uint loTrackNum = getTrackNum1(bb->loXY(_dir));
  if (loTrackNum > 0) {
    loTrackNum--;
  }

  uint hiTrackNum = getTrackNum1(bb->hiXY(_dir));

  int loXY = bb->loXY(d);
  int hiXY = bb->hiXY(d);
  uint loMarker = getBucketNum(loXY);
  uint hiMarker = getBucketNum(hiXY);

  for (uint ii = loTrackNum; ii <= hiTrackNum; ii++) {
    Ath__track* track = _trackTable[ii];
    if (track == nullptr) {
      continue;
    }

    wireIdTable.resetCnt();
    uint cnt1 = track->search1(loXY, hiXY, loMarker, hiMarker, &wireIdTable);
    if (cnt1 <= 0) {
      continue;
    }

    Ath__wire* w0 = _wirePoolPtr->get(wireIdTable.get(0));
    Ath__wire* w1 = w0->makeWire(wirePool, w0->_xy, w0->_len);

    if (g != nullptr) {
      g->placeWire(w1);
    }
    idtable->add(w1->_id);

    for (uint jj = 1; jj < cnt1; jj++) {
      Ath__wire* w = _wirePoolPtr->get(wireIdTable.get(jj));

      uint dist = w->_xy - (w1->_xy + w1->_len);
      if (dist <= gxy[d]) {
        w1->setXY(w1->_xy, w->_xy + w->_len - w1->_xy);
      } else  // start new
      {
        w1 = w0->makeWire(wirePool, w->_xy, w->_len);

        if (g != nullptr) {
          g->placeWire(w1);
        }
        idtable->add(w1->_id);
      }
    }
  }

  return idtable->getCnt();
}
void Ath__grid::getBuses(Ath__array1D<Ath__box*>* boxTable, uint width)
{
  Ath__array1D<Ath__wire*> wireTable(32);

  for (uint ii = 0; ii < _trackCnt; ii++) {
    if (_blockedTrackTable[ii] > 0) {
      continue;
    }

    Ath__track* track = _trackTable[ii];
    if (track == nullptr) {
      continue;
    }

    if (!(_schema > 0)) {
      continue;
    }

    uint height = _base + ii * _pitch;

    wireTable.resetCnt();
    track->getAllWires(&wireTable, _markerCnt);

    for (uint jj = 0; jj < wireTable.getCnt(); jj++) {
      Ath__wire* e = wireTable.get(jj);
      if (!e->isTileBus()) {
        continue;
      }

      Ath__box* bb = new Ath__box();
      if (_dir > 0) {
        bb->set(e->_xy, height, e->_xy + e->_len, height + width);
      } else {
        bb->set(height, e->_xy, height + width, e->_xy + e->_len);
      }

      bb->setLayer(_level);

      boxTable->add(bb);
    }
  }
}
Ath__wire* Ath__grid::getWire_Linear(uint id)
{
  for (uint ii = 0; ii < _trackCnt; ii++) {
    Ath__track* tr = _trackTable[ii];
    if (tr == nullptr) {
      continue;
    }

    Ath__wire* w = tr->getWire_Linear(_markerCnt, id);
    if (w != nullptr) {
      return w;
    }
  }
  return nullptr;
}
void Ath__grid::adjustOverlapMakerEnd()
{
  int TTTnewAdj = 1;
  Ath__track *track, *tstr;
  for (uint ii = 0; ii < _trackCnt; ii++) {
    track = _trackTable[ii];
    if (track == nullptr) {
      continue;
    }
    tstr = nullptr;
    bool tohi = true;
    while ((tstr = track->getNextSubTrack(tstr, tohi)) != nullptr) {
      if (TTTnewAdj) {
        tstr->adjustOverlapMakerEnd(_markerCnt, _start, _markerLen);
      } else {
        tstr->adjustOverlapMakerEnd(_markerCnt);
      }
    }
  }
}

bool Ath__grid::isOrdered(bool /* unused: ascending */, uint* cnt)
{
  bool ordered = true;
  for (uint ii = 0; ii < _trackCnt; ii++) {
    Ath__track* tr = _trackTable[ii];
    if (tr == nullptr) {
      continue;
    }

    if (!tr->isAscendingOrdered(_markerCnt, cnt)) {
      fprintf(stdout, "Track #%d is not ordered\n", ii);
      ordered = false;
    }
  }
  return ordered;
}
uint Ath__grid::getBucketNum(int xy)
{
  int offset = xy - _start;
  if (offset < 0) {
    return 0;
  }
  uint b = offset / _markerLen;
  if (b == 0) {
    return 0;
  }

  if (b >= _markerCnt) {
    return _markerCnt - 1;
  }
  return b;
}
uint Ath__grid::getWidth()
{
  return _width;
}

int Ath__grid::getXYbyWidth(int xy, uint* mark)
{
  int offset = xy - _start;
  if (offset < 0) {
    *mark = 0;
    return 0;
  }
  uint a = offset / _width;
  int b = a / _markerLen;
  if (b > 3) {
    *mark = 3;
  } else {
    *mark = b;
  }
  return a;
}
uint Ath__grid::getTrackNum1(int xy)
{
  int a = xy - _base;

  // if (a<0)
  if (xy < _base) {
    return 0;
  }

  uint b = a / _pitch;
  if (b >= _trackCnt) {
    return _trackCnt - 1;
  }
  return b;
}
uint Ath__grid::getTrackNum(int* ll, uint d, uint* marker)
{
  *marker = getBucketNum(ll[d]);

  int a = ll[_dir] - _base;

  if (a < 0) {
    return 0;
  }

  uint b = a / _pitch;
  if (b >= _trackCnt) {
    return _trackCnt - 1;
  }
  return b;
}
uint Ath__grid::getTrackNum(Ath__box* box)
{
  const odb::Rect rect = box->getRect();
  int ll[2] = {rect.xMin(), rect.yMin()};

  int a = ll[_dir] - _base;

  if (a < 0) {
    return 0;
  }
  return a / _pitch;
}
Ath__wire* Ath__grid::makeWire(uint dir,
                               int* ll,
                               int* ur,
                               uint id1,
                               uint id2,
                               uint type)
{
  Ath__wire* w = getPoolWire();
  w->_srcId = 0;
  w->_srcWire = nullptr;

  w->reset();
  w->set(dir, ll, ur);
  w->_boxId = id1;
  w->_otherId = id2;

  w->_flags = type;

  return w;
}
Ath__wire* Ath__grid::makeWire(int* ll, int* ur, uint id, uint* m1)
{
  uint d = (_dir > 0) ? 0 : 1;

  int xy1 = ll[d];
  // int xy2= ur[d];
  *m1 = getBucketNum(xy1);

  Ath__wire* w = getPoolWire();
  w->_srcId = 0;
  w->_otherId = 0;

  w->reset();
  w->set(_dir, ll, ur);
  w->_boxId = id;

  return w;
}
Ath__wire* Ath__grid::makeWire(Ath__box* box,
                               uint* id,
                               uint* m1,
                               uint* m2,
                               uint /* unused: fullTrack */)
{
  const odb::Rect rect = box->getRect();
  int ll[2] = {rect.xMin(), rect.yMin()};
  int ur[2] = {rect.xMax(), rect.yMax()};

  *m1 = 0;
  *m2 = 3;
  Ath__wire* w = getPoolWire();
  w->_otherId = 0;

  *id = w->_id;
  w->reset();
  w->set(_dir, ll, ur);
  w->_boxId = box->getId();
  w->_srcId = 0;
  return w;
}

uint Ath__grid::getFirstTrack(uint divider)
{
  int xy = _base + (_max - _base) / divider;

  return getAbsTrackNum(xy);
}
int Ath__grid::getClosestTrackCoord(int xy)
{
  int track1 = getAbsTrackNum(xy);
  int ii;
  for (ii = track1 - 1; ii < (int) _trackCnt; ii++) {
    if (_trackTable[ii] != nullptr) {
      break;
    }
  }
  int h1 = _max;
  if (ii < (int) _trackCnt) {
    h1 = getTrackHeight(ii);
  }

  for (ii = track1 + 1; ii >= 0; ii--) {
    if (_trackTable[ii] != nullptr) {
      break;
    }
  }
  int h2 = _base;
  if (ii > 0) {
    h2 = getTrackHeight(ii);
  }

  if (xy - h2 < h1 - xy) {
    return h2 + _width / 2;
  }
  return h1 + _width / 2;
}
int Ath__grid::findEmptyTrack(int ll[2], int ur[2])
{
  uint track1 = getAbsTrackNum(ll[_dir]);
  uint track2 = getAbsTrackNum(ur[_dir]);
  uint cnt = 0;
  for (uint ii = track1; ii <= track2; ii++) {
    if (_trackTable[ii] == nullptr) {
      cnt++;
      continue;
    }
    Ath__wire w;
    w.reset();

    int xy1 = (ll[_dir % 1] - _start) / _width;
    int xy2 = (ur[_dir % 1] - _start) / _width;

    w.set(_dir, ll, ur);

    int markIndex1 = xy1 / _markerLen;
    int markIndex2 = xy2 / _markerLen;

    if (_trackTable[ii]->overlapCheck(&w, markIndex1, markIndex2)) {
      continue;
    }
    cnt++;
  }
  if (cnt == track2 - track1 + 1) {
    return track1;
  }

  return -1;
}

void Ath__gridTable::init1(uint memChunk,
                           uint rowSize,
                           uint colSize,
                           uint dx,
                           uint dy)
{
  _trackPool = new AthPool<Ath__track>(memChunk);
  _wirePool = new AthPool<Ath__wire>(memChunk * 1000);

  _wirePool->alloc();  // so all wire ids>0

  _rowSize = rowSize;
  _colSize = colSize;
  _rowCnt = dy / rowSize + 1;
  _colCnt = dx / colSize + 1;

  _wireCnt = 0;
  resetMaxArea();
}

Ath__gridTable::Ath__gridTable(Ath__box* bb,
                               uint rowSize,
                               uint colSize,
                               uint layer,
                               uint dir,
                               uint width,
                               uint pitch)
{
  init1(1024, rowSize, colSize, bb->getDX(), bb->getDY());
  _bbox.set(bb);
  _schema = 0;
  _overlapTouchCheck = 1;
  _noPowerSource = 0;
  _noPowerTarget = 0;
  _CCshorts = 0;
  _CCtargetHighTracks = 1;
  _targetTrackReversed = false;
  _ccContextDepth = 0;
  _ccContextArray = nullptr;
  _allNet = true;
  _useDbSdb = true;
  _overlapAdjust = Z_noAdjust;
  _powerMultiTrackWire = 0;
  _signalMultiTrackWire = 0;
  _bandWire = nullptr;

  _gridTable = new Ath__grid**[_rowCnt];
  const odb::Rect rect = bb->getRect();
  int y1 = rect.yMin();
  for (uint ii = 0; ii < _rowCnt; ii++) {
    _gridTable[ii] = new Ath__grid*[_colCnt];

    int y2 = y1 + rowSize;
    int x1 = rect.xMin();
    for (uint jj = 0; jj < _colCnt; jj++) {
      int x2 = x1 + colSize;
      uint num = ii * 1000 + jj;

      Ath__box box;
      box.set(x1, y1, x2, y2);
      _gridTable[ii][jj] = new Ath__grid(
          this, _trackPool, _wirePool, &box, layer, dir, num, width, pitch, 32);

      x1 = x2;
    }
    y1 = y2;
  }
}
Ath__gridTable::Ath__gridTable(dbBox* bb,
                               uint rowSize,
                               uint colSize,
                               uint layer,
                               uint dir,
                               uint width,
                               uint pitch,
                               uint minWidth)
{
  init1(1024, rowSize, colSize, bb->getDX(), bb->getDY());
  _rectBB = bb->getBox();
  _schema = 1;
  _overlapTouchCheck = 1;
  _noPowerSource = 0;
  _noPowerTarget = 0;
  _CCshorts = 0;
  _CCtargetHighTracks = 1;
  _targetTrackReversed = false;
  _ccContextDepth = 0;
  _ccContextArray = nullptr;
  _allNet = true;
  _useDbSdb = true;
  _overlapAdjust = Z_noAdjust;
  _powerMultiTrackWire = 0;
  _signalMultiTrackWire = 0;
  _bandWire = nullptr;

  uint maxCellNumPerMarker = 16;
  uint markerCnt = (bb->getDX() / minWidth) / maxCellNumPerMarker;

  _gridTable = new Ath__grid**[_rowCnt];
  int y1 = bb->yMin();
  for (uint ii = 0; ii < _rowCnt; ii++) {
    _gridTable[ii] = new Ath__grid*[_colCnt];

    int y2 = y1 + rowSize;
    int x1 = bb->xMin();
    for (uint jj = 0; jj < _colCnt; jj++) {
      int x2 = x1 + colSize;
      uint num = ii * 1000 + jj;
      // Rect rectBB(x1, y1, x2, y2);
      _gridTable[ii][jj]
          = new Ath__grid(this, _trackPool, _wirePool, layer, num, markerCnt);

      _gridTable[ii][jj]->setTracks(dir, width, pitch, x1, y1, x2, y2);
      _gridTable[ii][jj]->setSchema(_schema);
      x1 = x2;
    }
    y1 = y2;
  }
}
void Ath__gridTable::releaseWire(uint wireId)
{
  Ath__wire* w = _wirePool->get(wireId);
  _wirePool->free(w);
}
Ath__wire* Ath__gridTable::getWirePtr(uint id)
{
  return _wirePool->get(id);
}
uint Ath__gridTable::getRowCnt()
{
  return _rowCnt;
}
uint Ath__gridTable::getColCnt()
{
  return _colCnt;
}
void Ath__gridTable::dumpTrackCounts(FILE* fp)
{
  fprintf(fp, "Multiple_track_power_wires : %d\n", _powerMultiTrackWire);
  fprintf(fp, "Multiple_track_signal_wires : %d\n", _signalMultiTrackWire);
  fprintf(fp,
          "layer  dir   alloc    live offbase  expand  tsubtn   toptk  stn\n");
  Ath__grid* tgrid;
  uint topBigTrack = 0;
  uint topSubtNum;
  uint totalSubtNum;
  uint expTrackNum;
  int trn;
  uint offbase;
  uint liveCnt;
  uint talloc = 0;
  uint tlive = 0;
  uint toffbase = 0;
  uint texpand = 0;
  uint ttsubtn = 0;
  for (uint layer = 1; layer < _colCnt; layer++) {
    for (uint dir = 0; dir < _rowCnt; dir++) {
      topBigTrack = 0;
      topSubtNum = 0;
      totalSubtNum = 0;
      expTrackNum = 0;
      offbase = 0;
      liveCnt = 0;
      tgrid = _gridTable[dir][layer];
      for (trn = 0; trn < (int) tgrid->_trackCnt; trn++) {
        if (tgrid->_trackTable[trn] == nullptr) {
          continue;
        }
        liveCnt++;
        if (tgrid->_base + tgrid->_pitch * trn
            != tgrid->_trackTable[trn]->_base) {
          offbase++;
        }
        if (tgrid->_subTrackCnt[trn] == 0) {
          continue;
        }
        expTrackNum++;
        totalSubtNum += tgrid->_subTrackCnt[trn];
        if (tgrid->_subTrackCnt[trn] > topSubtNum) {
          topSubtNum = tgrid->_subTrackCnt[trn];
          topBigTrack = trn;
        }
      }
      fprintf(fp,
              "%5d%5d%8d%8d%8d%8d%8d%8d%5d\n",
              layer,
              dir,
              tgrid->_trackCnt,
              liveCnt,
              offbase,
              expTrackNum,
              totalSubtNum,
              topBigTrack,
              topSubtNum);
      talloc += tgrid->_trackCnt;
      tlive += liveCnt;
      toffbase += offbase;
      texpand += expTrackNum;
      ttsubtn += totalSubtNum;
    }
  }
  fprintf(fp,
          "---------------------------------------------------------------\n");
  fprintf(fp,
          "          %8d%8d%8d%8d%8d\n",
          talloc,
          tlive,
          toffbase,
          texpand,
          ttsubtn);
}
Ath__gridTable::Ath__gridTable(Rect* bb,
                               uint rowCnt,
                               uint colCnt,
                               uint* /* unused: width */,
                               uint* pitch,
                               uint* /* unused: spacing */,
                               const int* X1,
                               const int* Y1)
{
  // for net wires
  init1(1024, bb->dy(), bb->dx(), bb->dx(), bb->dy());
  _rectBB.reset(bb->xMin(), bb->yMin(), bb->xMax(), bb->yMax());
  _rowCnt = rowCnt;
  _colCnt = colCnt;
  _schema = 1;
  _overlapTouchCheck = 1;
  _noPowerSource = 0;
  _noPowerTarget = 0;
  _CCshorts = 0;
  _CCtargetHighTracks = 1;
  _targetTrackReversed = false;
  _ccContextDepth = 0;
  _ccContextArray = nullptr;
  _allNet = true;
  _useDbSdb = true;
  _overlapAdjust = Z_noAdjust;
  _powerMultiTrackWire = 0;
  _signalMultiTrackWire = 0;
  _bandWire = nullptr;

  uint markerLen = 500000;  // EXT-DEFAULT

  // int x1= bb->xMin();
  // int y1= bb->yMin();
  int x1, y1;
  int x2 = bb->xMax();
  int y2 = bb->yMax();

  _gridTable = new Ath__grid**[_rowCnt];
  for (uint ii = 0; ii < _rowCnt; ii++) {
    _gridTable[ii] = new Ath__grid*[_colCnt];
    _gridTable[ii][0] = nullptr;

    for (uint jj = 1; jj < _colCnt; jj++) {
      uint num = ii * 1000 + jj;

      _gridTable[ii][jj]
          = new Ath__grid(this, _trackPool, _wirePool, jj, num, 10);
      x1 = X1 ? X1[jj] : bb->xMin();
      y1 = Y1 ? Y1[jj] : bb->yMin();
      _gridTable[ii][jj]->setTracks(
          ii, 1, pitch[jj], x1, y1, x2, y2, markerLen);
      _gridTable[ii][jj]->setSchema(_schema);
    }
  }
}

Ath__gridTable::Ath__gridTable(Rect* bb,
                               uint layer,
                               uint dir,
                               uint width,
                               uint pitch,
                               uint minWidth)
{
  init1(1024, bb->dy(), bb->dx(), bb->dx(), bb->dy());
  _colCnt = 1;
  _rowCnt = 1;
  _rectBB.reset(bb->xMin(), bb->yMin(), bb->xMax(), bb->yMax());
  _schema = 1;
  _overlapTouchCheck = 1;
  _noPowerSource = 0;
  _noPowerTarget = 0;
  _CCshorts = 0;
  _CCtargetHighTracks = 1;
  _targetTrackReversed = false;
  _ccContextDepth = 0;
  _ccContextArray = nullptr;
  _allNet = true;
  _useDbSdb = true;
  _overlapAdjust = Z_noAdjust;
  _powerMultiTrackWire = 0;
  _signalMultiTrackWire = 0;
  _bandWire = nullptr;

  uint maxCellNumPerMarker = 16;
  uint markerCnt = (bb->dx() / minWidth) / maxCellNumPerMarker;
  if (markerCnt == 0) {
    markerCnt = 1;
  }

  Ath__grid* g
      = new Ath__grid(this, _trackPool, _wirePool, layer, 1, markerCnt);

  g->setTracks(
      dir, width, pitch, bb->xMin(), bb->yMin(), bb->xMax(), bb->yMax());
  g->setSchema(_schema);

  _gridTable = new Ath__grid**[1];
  _gridTable[0] = new Ath__grid*[1];

  _gridTable[0][0] = g;
}
Ath__gridTable::~Ath__gridTable()
{
  delete _trackPool;
  delete _wirePool;

  for (uint ii = 0; ii < _rowCnt; ii++) {
    for (uint jj = 0; jj < _rowCnt; jj++) {
      delete _gridTable[ii][jj];
    }
    delete[] _gridTable[ii];
  }
  delete[] _gridTable;
}
int Ath__gridTable::xMin()
{
  if (_schema > 0) {
    return _rectBB.xMin();
  }
  return _bbox.getRect().xMin();
}
int Ath__gridTable::xMax()
{
  if (_schema > 0) {
    return _rectBB.xMax();
  }
  return _bbox.getRect().xMax();
}
int Ath__gridTable::yMin()
{
  if (_schema > 0) {
    return _rectBB.yMin();
  }
  return _bbox.getRect().yMin();
}
int Ath__gridTable::yMax()
{
  if (_schema > 0) {
    return _rectBB.yMax();
  }
  return _bbox.getRect().yMax();
}
uint Ath__gridTable::getRowNum(int y)
{
  int dy = y - yMin();
  if (dy < 0) {
    return 0;
  }
  return dy / _rowSize;
}
uint Ath__gridTable::getColNum(int x)
{
  int dx = x - xMin();
  if (dx < 0) {
    return 0;
  }

  return dx / _colSize;
}
uint Ath__gridTable::search(Ath__searchBox* bb,
                            uint row,
                            uint col,
                            Ath__array1D<uint>* idTable,
                            bool wireIdFlag)
{
  return _gridTable[row][col]->search(bb, idTable, wireIdFlag);
}
uint Ath__gridTable::search(Ath__searchBox* bb,
                            uint* gxy,
                            uint row,
                            uint col,
                            Ath__array1D<uint>* idtable,
                            Ath__grid* g)
{
  return _gridTable[row][col]->search(bb, gxy, idtable, g);
}
uint Ath__gridTable::search(Ath__searchBox* bb, Ath__array1D<uint>* idTable)
{
  uint row1 = getRowNum(bb->loXY(1));
  if (row1 > 0) {
    row1--;
  }

  uint row2 = getRowNum(bb->hiXY(1));

  uint col1 = getColNum(bb->loXY(0));
  if (col1 > 0) {
    col1--;
  }

  uint col2 = getColNum(bb->hiXY(0));

  for (uint ii = row1; ii < _rowCnt && ii <= row2; ii++) {
    for (uint jj = col1; jj < _colCnt && jj <= col2; jj++) {
      _gridTable[ii][jj]->search(bb, idTable);
    }
  }
  return 0;
}
bool Ath__gridTable::getRowCol(int x1, int y1, uint* row, uint* col)
{
  *row = getRowNum(y1);
  if (*row >= _rowCnt) {
    fprintf(stderr, "Y=%d Out of Row Range %d\n", y1, _rowCnt);
    return false;
  }
  *col = getColNum(x1);
  if (*col >= _colCnt) {
    fprintf(stderr, "X=%d Out of Col Range %d\n", x1, _colCnt);
    return false;
  }
  return true;
}
uint Ath__gridTable::setExtrusionMarker(uint startRow, uint startCol)
{
  uint cnt = 0;
  for (uint ii = startRow; ii < _rowCnt; ii++) {
    for (uint jj = startCol; jj < _colCnt; jj++) {
      cnt += _gridTable[ii][jj]->setExtrusionMarker();
    }
  }
  return cnt;
}
AthPool<Ath__wire>* Ath__grid::getWirePoolPtr()
{
  return _wirePoolPtr;
}

uint Ath__grid::removeMarkedNetWires()
{
  uint cnt = 0;
  for (uint ii = 0; ii < _trackCnt; ii++) {
    Ath__track* btrack = _trackTable[ii];
    if (btrack == nullptr) {
      continue;
    }

    Ath__track* track = nullptr;
    bool tohi = true;
    while ((track = btrack->getNextSubTrack(track, tohi)) != nullptr) {
      cnt += track->removeMarkedNetWires();
    }
  }
  return cnt;
}

Ath__grid* Ath__gridTable::getGrid(uint row, uint col)
{
  return _gridTable[row][col];
}
bool Ath__gridTable::addBox(uint row, uint col, dbBox* bb)
{
  Ath__grid* g = _gridTable[row][col];

  g->placeBox(bb, 0, 0);

  return true;
}
Ath__wire* Ath__gridTable::addBox(dbBox* bb, uint wtype, uint id)
{
  uint row = 0;
  uint col = 0;
  Ath__grid* g = _gridTable[row][col];

  g->placeBox(bb, wtype, id);

  return nullptr;
}
Ath__wire* Ath__gridTable::addBox(Ath__box* bb)
{
  uint row;
  uint col;
  const odb::Rect rect = bb->getRect();
  if (!getRowCol(rect.xMin(), rect.yMin(), &row, &col)) {
    return nullptr;
  }

  Ath__grid* g = _gridTable[row][col];
  g->placeBox(bb);

  return nullptr;
}
Ath__wire* Ath__gridTable::getWire_Linear(uint instId)
{
  // bool ordered= true;

  // uint cnt= 0;
  for (uint ii = 0; ii < _rowCnt; ii++) {
    for (uint jj = 0; jj < _colCnt; jj++) {
      Ath__wire* w = _gridTable[ii][jj]->getWire_Linear(instId);
      if (w != nullptr) {
        return w;
      }
    }
  }
  return nullptr;
}
void Ath__gridTable::adjustOverlapMakerEnd()
{
  if (_overlapAdjust != Z_endAdjust) {
    return;
  }
  for (uint ii = 0; ii < _rowCnt; ii++) {
    for (uint jj = 0; jj < _colCnt; jj++) {
      if (_gridTable[ii][jj]) {
        _gridTable[ii][jj]->adjustOverlapMakerEnd();
      }
    }
  }
}

void Ath__gridTable::incrNotAlignedOverlap(Ath__wire* w1, Ath__wire* w2)
{
  if (w1->isPower() != w2->isPower()) {
    _signalPowerNotAlignedOverlap++;
  } else if (w1->isPower()) {
    _powerNotAlignedOverlap++;
  } else {
    _signalNotAlignedOverlap++;
  }
}
void Ath__gridTable::incrSignalOverlap()
{
  _signalOverlap++;
}
void Ath__gridTable::incrPowerOverlap()
{
  _powerOverlap++;
}
void Ath__gridTable::incrSignalToPowerOverlap()
{
  _signalPowerOverlap++;
}
void Ath__gridTable::incrPowerToSignallOverlap()
{
  _powerSignalOverlap++;
}
void Ath__gridTable::incrMultiTrackWireCnt(bool isPower)
{
  if (isPower) {
    _powerMultiTrackWire++;
  } else {
    _signalMultiTrackWire++;
  }
}
bool Ath__gridTable::isOrdered(bool /* unused: ascending */)
{
  bool ordered = true;

  for (uint ii = 0; ii < _rowCnt; ii++) {
    for (uint jj = 0; jj < _colCnt; jj++) {
      uint cnt1 = 0;
      if (!_gridTable[ii][jj]->isOrdered(true, &cnt1)) {
        ordered = false;
        fprintf(stdout,
                "NOT ordered grid [%d][%d] -- has %d wires\n",
                ii,
                jj,
                cnt1);
      } else {
        fprintf(
            stdout, "Ordered grid [%d][%d] -- has %d wires\n", ii, jj, cnt1);
      }
    }
  }
  return ordered;
}

void Ath__gridTable::removeMarkedNetWires()
{
  uint cnt = 0;
  for (uint jj = 1; jj < _colCnt; jj++) {
    for (int ii = _rowCnt - 1; ii >= 0; ii--) {
      Ath__grid* netGrid = _gridTable[ii][jj];
      cnt += netGrid->removeMarkedNetWires();
    }
  }
  fprintf(stdout, "remove %d sdb wires.\n", cnt);
}

void Ath__gridTable::setExtControl(dbBlock* block,
                                   bool useDbSdb,
                                   uint adj,
                                   uint npsrc,
                                   uint nptgt,
                                   uint ccUp,
                                   bool allNet,
                                   uint contextDepth,
                                   Ath__array1D<int>** contextArray,
                                   Ath__array1D<SEQ*>*** dgContextArray,
                                   uint* dgContextDepth,
                                   uint* dgContextPlanes,
                                   uint* dgContextTracks,
                                   uint* dgContextBaseLvl,
                                   int* dgContextLowLvl,
                                   int* dgContextHiLvl,
                                   uint* dgContextBaseTrack,
                                   int* dgContextLowTrack,
                                   int* dgContextHiTrack,
                                   int** dgContextTrackBase,
                                   AthPool<SEQ>* seqPool)
{
  _block = block;
  _useDbSdb = useDbSdb;
  _overlapAdjust = adj;
  _noPowerSource = npsrc;
  _noPowerTarget = nptgt;
  _CCtargetHighTracks = ccUp;
  if (ccUp == 2) {
    _CCtargetHighMarkedNet = 1;
  } else {
    _CCtargetHighMarkedNet = 0;
  }
  _targetTrackReversed = false;
  _ccContextDepth = contextDepth;
  _ccContextArray = contextArray;
  _allNet = allNet;
  _dgContextArray = dgContextArray;
  _dgContextDepth = dgContextDepth;
  _dgContextPlanes = dgContextPlanes;
  _dgContextTracks = dgContextTracks;
  _dgContextBaseLvl = dgContextBaseLvl;
  _dgContextLowLvl = dgContextLowLvl;
  _dgContextHiLvl = dgContextHiLvl;
  _dgContextBaseTrack = dgContextBaseTrack;
  _dgContextLowTrack = dgContextLowTrack;
  _dgContextHiTrack = dgContextHiTrack;
  _dgContextTrackBase = dgContextTrackBase;
  _seqPool = seqPool;
}
void Ath__gridTable::setExtControl_v2(dbBlock* block,
                                      bool useDbSdb,
                                      uint adj,
                                      uint npsrc,
                                      uint nptgt,
                                      uint ccUp,
                                      bool allNet,
                                      uint contextDepth,
                                      Ath__array1D<int>** contextArray,
                                      uint* contextLength,
                                      Ath__array1D<SEQ*>*** dgContextArray,
                                      uint* dgContextDepth,
                                      uint* dgContextPlanes,
                                      uint* dgContextTracks,
                                      uint* dgContextBaseLvl,
                                      int* dgContextLowLvl,
                                      int* dgContextHiLvl,
                                      uint* dgContextBaseTrack,
                                      int* dgContextLowTrack,
                                      int* dgContextHiTrack,
                                      int** dgContextTrackBase,
                                      AthPool<SEQ>* seqPool)
{
  _block = block;
  _useDbSdb = useDbSdb;
  _overlapAdjust = adj;
  _noPowerSource = npsrc;
  _noPowerTarget = nptgt;
  _CCtargetHighTracks = ccUp;
  if (ccUp == 2)
    _CCtargetHighMarkedNet = 1;
  else
    _CCtargetHighMarkedNet = 0;
  _targetTrackReversed = false;
  _ccContextDepth = contextDepth;
  _ccContextArray = contextArray;
  _ccContextLength = contextLength;
  _allNet = allNet;
  _dgContextArray = dgContextArray;
  _dgContextDepth = dgContextDepth;
  _dgContextPlanes = dgContextPlanes;
  _dgContextTracks = dgContextTracks;
  _dgContextBaseLvl = dgContextBaseLvl;
  _dgContextLowLvl = dgContextLowLvl;
  _dgContextHiLvl = dgContextHiLvl;
  _dgContextBaseTrack = dgContextBaseTrack;
  _dgContextLowTrack = dgContextLowTrack;
  _dgContextHiTrack = dgContextHiTrack;
  _dgContextTrackBase = dgContextTrackBase;
  _seqPool = seqPool;
}
void Ath__gridTable::reverseTargetTrack()
{
  _CCtargetHighTracks = _CCtargetHighTracks == 2 ? 0 : 2;
  _targetTrackReversed = _targetTrackReversed ? false : true;
}

void Ath__gridTable::setMaxArea(int x1, int y1, int x2, int y2)
{
  _maxSearchBox.set(x1, y1, x2, y2, 1);
  _setMaxArea = true;
}
void Ath__gridTable::resetMaxArea()
{
  _setMaxArea = false;
  _maxSearchBox.invalidateBox();
}
void Ath__gridTable::getBox(uint wid,
                            int* x1,
                            int* y1,
                            int* x2,
                            int* y2,
                            uint* level,
                            uint* id1,
                            uint* id2,
                            uint* wtype)
{
  Ath__wire* w = getWirePtr(wid);

  *id1 = w->_boxId;
  *id2 = w->_otherId;
  *wtype = w->_flags;
  *level = w->_track->getGrid()->getLevel();
  uint dir;
  w->getCoords(x1, y1, x2, y2, &dir);
}
uint Ath__gridTable::addBox(int x1,
                            int y1,
                            int x2,
                            int y2,
                            uint level,
                            uint id1,
                            uint id2,
                            uint wireType)
{
  Ath__searchBox bb(x1, y1, x2, y2, level);
  bb.setOwnerId(id1, id2);
  bb.setType(wireType);

  uint dir = bb.getDir();
  uint trackNum = !_v2 ? getGrid(dir, level)->placeWire(&bb)
                       : getGrid(dir, level)->placeWire_v2(&bb);
  _wireCnt++;
  return trackNum;
}
uint Ath__gridTable::getWireCnt()
{
  return _wireCnt;
}
uint Ath__gridTable::search(int x1,
                            int y1,
                            int x2,
                            int y2,
                            uint row,
                            uint col,
                            Ath__array1D<uint>* idTable,
                            bool /* unused: wireIdFlag */)
{
  Ath__searchBox bb(x1, y1, x2, y2, col, row);

  return search(&bb, row, col, idTable, true);  // single grid
}
void Ath__gridTable::getCoords(Ath__searchBox* bb, uint wireId)
{
  Ath__wire* w = getWirePtr(wireId);
  w->getCoords(bb);
}

}  // namespace rcx
