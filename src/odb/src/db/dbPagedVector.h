// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include "odb/ZException.h"
#include "odb/dbStream.h"
#include "odb/odb.h"

namespace odb {

template <class T, const uint P, const uint S>
class dbPagedVector;

//
// Vector - Creates a vector of type T. However, the vector is created
// as a series of pages of type T. The size of the page is specified
// by the template params.
//
// page_size = number of objects per page (MUST BE POWER OF TWO)
// page_shift = log2(page_size)
//
template <class T, const uint page_size = 128, const uint page_shift = 7>
class dbPagedVector
{
 private:
  T** _pages;
  unsigned int _page_cnt;
  unsigned int _page_tbl_size;
  unsigned int _next_idx;

  int _freedIdxHead;  // DKF - to delete
  int _freedIdxTail;  // DKF - to delete

  void resizePageTbl();
  void newPage();

 public:
  dbPagedVector();
  dbPagedVector(const dbPagedVector<T, page_size, page_shift>& V);
  ~dbPagedVector();

  void push_back(const T& item);

  uint push_back(int cnt, const T& item)
  {
    uint id = _next_idx;
    uint i;
    for (i = 0; i < cnt; ++i) {
      push_back(item);
    }
    return id;
  }

  unsigned int size() const { return _next_idx; }
  unsigned int getIdx(uint chunkSize, const T& ival);  // DKF - to delete
  void freeIdx(uint idx);                              // DKF - to delete
  void clear();

  T& operator[](unsigned int id)
  {
    ZASSERT(id < _next_idx);
    unsigned int page = (id & ~(page_size - 1)) >> page_shift;
    unsigned int offset = id & (page_size - 1);
    return _pages[page][offset];
  }
  const T& operator[](unsigned int id) const
  {
    ZASSERT(id < _next_idx);
    unsigned int page = (id & ~(page_size - 1)) >> page_shift;
    unsigned int offset = id & (page_size - 1);
    return _pages[page][offset];
  }

  bool operator==(const dbPagedVector<T, page_size, page_shift>& rhs) const;
  bool operator!=(const dbPagedVector<T, page_size, page_shift>& rhs) const
  {
    return !operator==(rhs);
  }
};

template <class T, const uint P, const uint S>
unsigned int dbPagedVector<T, P, S>::getIdx(uint chunkSize, const T& ival)
{
  uint idx;
  if (_freedIdxHead == -1) {
    idx = _next_idx;
    _next_idx += chunkSize;
    unsigned int page = ((_next_idx - 1) & ~(P - 1)) >> S;
    if (page == _page_cnt) {
      newPage();
    }
  } else {
    idx = (uint) _freedIdxHead;
    if (idx == (uint) _freedIdxTail) {
      _freedIdxHead = -1;
    } else {
      unsigned int page = (idx & ~(P - 1)) >> S;
      unsigned int offset = idx & (P - 1);
      uint* fidxp = (uint*) (&_pages[page][offset]);
      _freedIdxHead = (int) *fidxp;
    }
  }
  for (uint dd = 0; dd < chunkSize; dd++) {
    uint id = idx + dd;
    ZASSERT(id < _next_idx);
    unsigned int page = (id & ~(P - 1)) >> S;
    unsigned int offset = id & (P - 1);
    _pages[page][offset] = ival;
  }
  return idx;
}

template <class T, const uint P, const uint S>
void dbPagedVector<T, P, S>::freeIdx(uint idx)
{
  if (_freedIdxHead == -1) {
    _freedIdxHead = idx;
    _freedIdxTail = idx;
  } else {
    unsigned int page = (_freedIdxTail & ~(P - 1)) >> S;
    unsigned int offset = _freedIdxTail & (P - 1);
    uint* fidxp = (uint*) (&_pages[page][offset]);
    *fidxp = idx;
    _freedIdxTail = idx;
  }
}

template <class T, const uint P, const uint S>
dbPagedVector<T, P, S>::dbPagedVector()
{
  _pages = nullptr;
  _page_cnt = 0;
  _page_tbl_size = 0;
  _next_idx = 0;
  _freedIdxHead = -1;
}

template <class T, const uint P, const uint S>
dbPagedVector<T, P, S>::dbPagedVector(const dbPagedVector<T, P, S>& V)
{
  _pages = nullptr;
  _page_cnt = 0;
  _page_tbl_size = 0;
  _next_idx = 0;
  _freedIdxHead = -1;
  uint sz = V.size();

  uint i;
  for (i = 0; i < sz; ++i) {
    push_back(V[i]);
  }
}

template <class T, const uint P, const uint S>
void dbPagedVector<T, P, S>::clear()
{
  if (_pages) {
    unsigned int i;

    for (i = 0; i < _page_cnt; ++i) {
      delete[] _pages[i];
    }

    delete[] _pages;
  }

  _pages = nullptr;
  _page_cnt = 0;
  _page_tbl_size = 0;
  _next_idx = 0;
  _freedIdxHead = -1;
}

template <class T, const uint P, const uint S>
dbPagedVector<T, P, S>::~dbPagedVector()
{
  clear();
}

template <class T, const uint P, const uint S>
void dbPagedVector<T, P, S>::resizePageTbl()
{
  T** old_tbl = _pages;
  unsigned int old_tbl_size = _page_tbl_size;

  if (_page_tbl_size == 1) {
    ++_page_tbl_size;
  } else {
    _page_tbl_size += (unsigned int) ((float) _page_tbl_size * (0.5));
  }

  _pages = new T*[_page_tbl_size];

  unsigned int i;

  for (i = 0; i < old_tbl_size; ++i) {
    _pages[i] = old_tbl[i];
  }

  for (; i < _page_tbl_size; ++i) {
    _pages[i] = nullptr;
  }

  delete[] old_tbl;
}

template <class T, const uint P, const uint S>
void dbPagedVector<T, P, S>::newPage()
{
  T* page = new T[P];

  if (_page_tbl_size == 0) {
    _pages = new T*[1];
    _page_tbl_size = 1;
  } else if (_page_tbl_size == _page_cnt) {
    resizePageTbl();
  }

  _pages[_page_cnt] = page;
  ++_page_cnt;
}

template <class T, const uint P, const uint S>
void dbPagedVector<T, P, S>::push_back(const T& item)
{
  unsigned int page = (_next_idx & ~(P - 1)) >> S;

  if (page == _page_cnt) {
    newPage();
  }

  unsigned int offset = _next_idx & (P - 1);
  ++_next_idx;

  T* objects = _pages[page];
  objects[offset] = item;
}

template <class T, const uint P, const uint S>
inline bool dbPagedVector<T, P, S>::operator==(
    const dbPagedVector<T, P, S>& rhs) const
{
  uint sz = size();

  if (sz != rhs.size()) {
    return false;
  }

  uint i;
  for (i = 0; i < sz; ++i) {
    const T& l = (*this)[i];
    const T& r = rhs[i];

    if (l != r) {
      return false;
    }
  }

  return true;
}

template <class T, const uint P, const uint S>
inline dbOStream& operator<<(dbOStream& stream, const dbPagedVector<T, P, S>& v)
{
  uint sz = v.size();
  stream << sz;

  uint i;
  for (i = 0; i < sz; ++i) {
    const T& t = v[i];
    stream << t;
  }

  return stream;
}

template <class T, const uint P, const uint S>
inline dbIStream& operator>>(dbIStream& stream, dbPagedVector<T, P, S>& v)
{
  v.clear();

  uint sz;
  stream >> sz;
  T t;
  uint i;

  for (i = 0; i < sz; ++i) {
    stream >> t;
    v.push_back(t);
  }

  return stream;
}

}  // namespace odb
