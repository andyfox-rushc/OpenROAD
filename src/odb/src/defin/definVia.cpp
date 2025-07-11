// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "definVia.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>

#include "odb/db.h"
#include "odb/dbShape.h"
#include "utl/Logger.h"

namespace odb {

void definVia::viaBegin(const char* name)
{
  assert(_cur_via == nullptr);

  _cur_via = dbVia::create(_block, name);

  if (_cur_via == nullptr) {
    _logger->warn(utl::ODB, 166, "error: duplicate via ({})", name);
    ++_errors;
  }
}

void definVia::viaRule(const char* rule)
{
  if (_cur_via == nullptr) {
    return;
  }

  dbTechViaGenerateRule* viarule = _tech->findViaGenerateRule(rule);

  if (viarule == nullptr) {
    _logger->warn(utl::ODB,
                  167,
                  "error: cannot file VIA GENERATE rule in technology ({}).",
                  rule);
    ++_errors;
    return;
  }

  _cur_via->setViaGenerateRule(viarule);
}

void definVia::viaCutSize(int xSize, int ySize)
{
  if (_cur_via == nullptr) {
    return;
  }

  if (_params == nullptr) {
    _params = new dbViaParams();
  }

  _params->setXCutSize(dbdist(xSize));
  _params->setYCutSize(dbdist(ySize));
}

bool definVia::viaLayers(const char* botName,
                         const char* cutName,
                         const char* topName)
{
  if (_cur_via == nullptr) {
    return false;
  }

  if (_params == nullptr) {
    _params = new dbViaParams();
  }

  dbTechLayer* bot = _tech->findLayer(botName);

  if (bot == nullptr) {
    _logger->warn(
        utl::ODB, 168, "error: undefined layer ({}) referenced", botName);
    ++_errors;
    return false;
  }

  dbTechLayer* cut = _tech->findLayer(cutName);

  if (cut == nullptr) {
    _logger->warn(
        utl::ODB, 169, "error: undefined layer ({}) referenced", cutName);
    ++_errors;
    return false;
  }

  dbTechLayer* top = _tech->findLayer(topName);

  if (top == nullptr) {
    _logger->warn(
        utl::ODB, 170, "error: undefined layer ({}) referenced", topName);
    ++_errors;
    return false;
  }

  _params->setTopLayer(top);
  _params->setBottomLayer(bot);
  _params->setCutLayer(cut);
  return true;
}

void definVia::viaCutSpacing(int xSpacing, int ySpacing)
{
  if (_cur_via == nullptr) {
    return;
  }

  if (_params == nullptr) {
    _params = new dbViaParams();
  }

  _params->setXCutSpacing(dbdist(xSpacing));
  _params->setYCutSpacing(dbdist(ySpacing));
}

void definVia::viaEnclosure(int xBot, int yBot, int xTop, int yTop)
{
  if (_cur_via == nullptr) {
    return;
  }

  if (_params == nullptr) {
    _params = new dbViaParams();
  }

  _params->setXBottomEnclosure(dbdist(xBot));
  _params->setYBottomEnclosure(dbdist(yBot));
  _params->setXTopEnclosure(dbdist(xTop));
  _params->setYTopEnclosure(dbdist(yTop));
}

void definVia::viaRowCol(int numCutRows, int numCutCols)
{
  if (_cur_via == nullptr) {
    return;
  }

  if (_params == nullptr) {
    _params = new dbViaParams();
  }

  _params->setNumCutRows(numCutRows);
  _params->setNumCutCols(numCutCols);
}

void definVia::viaOrigin(int x, int y)
{
  if (_cur_via == nullptr) {
    return;
  }

  if (_params == nullptr) {
    _params = new dbViaParams();
  }

  _params->setXOrigin(dbdist(x));
  _params->setYOrigin(dbdist(y));
}

void definVia::viaOffset(int xBot, int yBot, int xTop, int yTop)
{
  if (_cur_via == nullptr) {
    return;
  }

  if (_params == nullptr) {
    _params = new dbViaParams();
  }

  _params->setXBottomOffset(dbdist(xBot));
  _params->setYBottomOffset(dbdist(yBot));
  _params->setXTopOffset(dbdist(xTop));
  _params->setYTopOffset(dbdist(yTop));
}

void definVia::viaPattern(const char* pattern)
{
  if (_cur_via == nullptr) {
    return;
  }

  _cur_via->setPattern(pattern);
}

void definVia::viaRect(const char* layer_name, int x1, int y1, int x2, int y2)
{
  if (_cur_via == nullptr) {
    return;
  }

  x1 = dbdist(x1);
  y1 = dbdist(y1);
  x2 = dbdist(x2);
  y2 = dbdist(y2);

  dbTechLayer* layer = _tech->findLayer(layer_name);

  if (layer == nullptr) {
    _logger->warn(
        utl::ODB, 171, "error: undefined layer ({}) referenced", layer_name);
    ++_errors;
    return;
  }

  dbBox::create(_cur_via, layer, x1, y1, x2, y2);
}

void definVia::viaEnd()
{
  if (_cur_via == nullptr) {
    return;
  }

  if (_params) {
    _cur_via->setViaParams(*_params);
    delete _params;
    _params = nullptr;
  }

  dbSet<dbBox> boxes = _cur_via->getBoxes();

  if (boxes.reversible() && boxes.orderReversed()) {
    boxes.reverse();
  }

  if (boxes.size() < 3) {
    _logger->error(utl::ODB,
                   299,
                   "Via {} has only {} shapes and must have at least three.",
                   _cur_via->getName(),
                   boxes.size());
  }

  dbTechLayer* top = _cur_via->getTopLayer();
  dbTechLayerType top_type = top->getType();
  if (top_type == dbTechLayerType::CUT) {
    _logger->error(utl::ODB,
                   300,
                   "Via {} has cut top layer {}",
                   _cur_via->getName(),
                   top->getName());
  }

  dbTechLayer* bottom = _cur_via->getBottomLayer();
  dbTechLayerType bottom_type = bottom->getType();
  if (bottom_type == dbTechLayerType::CUT) {
    _logger->error(utl::ODB,
                   301,
                   "Via {} has cut bottom layer {}",
                   _cur_via->getName(),
                   bottom->getName());
  }

  bool has_cut = false;
  for (dbBox* box : boxes) {
    dbTechLayer* layer = box->getTechLayer();
    if (layer->getType() == dbTechLayerType::CUT) {
      has_cut = true;
      break;
    }
  }
  if (!has_cut) {
    _logger->error(
        utl::ODB, 302, "Via {} has no cut shapes.", _cur_via->getName());
  }

  _cur_via = nullptr;
}

}  // namespace odb
