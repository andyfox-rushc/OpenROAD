// EcoEnvironment.cpp (simplified implementation)
#include "EcoEnvironment.h"

#include "odb/lefout.h"  // For writing DEF if needed

namespace rl_eco {

EcoEnvironment::EcoEnvironment(const EnvironmentConfig& config)
    : current_step_(0), max_steps_(config.max_steps)
{
  /*
  db_ = odb::dbDatabase::create();
  db_->readLef(config.lef_file);
  db_->readDef(config.def_file);

  sta_ = sta::Sta::getOrCreateSta();
  sta_->readLiberty(config.liberty_file.c_str());
  sta_->makeClkNetwork("clk", 10.0);  // Example clock; customize

  spare_cells_ = config.spare_cells;
  // Initialize spares, etc.
  */
}

EcoEnvironment::~EcoEnvironment()
{
  delete sta_;
  odb::dbDatabase::destroy(db_);
}

torch::Tensor EcoEnvironment::reset()
{
  current_step_ = 0;
  sta_->updateTiming(true);  // Full timing update
  return getObservation();
}

std::tuple<torch::Tensor, float, bool> EcoEnvironment::step(int action)
{
  applyEco(action);
  sta_->updateTiming(false);  // Incremental update for speed

  auto obs = getObservation();
  float reward = computeReward();
  bool done = (current_step_ >= max_steps_) || isTimingClosed();
  current_step_++;
  return {obs, reward, done};
}

torch::Tensor EcoEnvironment::getObservation()
{
  // Example: Vector of 10 floats (slacks for top paths + spare util)
  std::vector<float> state(10, 0.0f);
  // Populate from sta_->reportTiming() or direct API (parse worst slacks)
  float wns = sta_->worstSlack(
      sta::MinMax::min() /*sta::MinMax::min()*/);  // Example API call
  state[0] = wns;

  // TODO
  //   state[1] = static_cast<float>(spare_cells_.size() - /* used count */);
  //  ... fill others

  return torch::tensor(state);
}

void EcoEnvironment::applyEco(int action)
{
  // Decode action (e.g., spare_idx = action / 5; type = action % 5)
  int spare_idx = action / 5;
  if (spare_idx >= spare_cells_.size()) {
    return;  // Invalid
  }

  auto block = db_->getChip()->getBlock();
  auto spare_inst = block->findInst(spare_cells_[spare_idx].c_str());
  // Example: Insert as buffer on critical net
  auto critical_net = block->findNet("critical_net");  // Identify from STA
  // Use odb API to connect/replace (e.g., odb::dbNet::connect, etc.)
  // Implement based on your ECO logic
}

float EcoEnvironment::computeReward()
{
  float new_wns = sta_->worstSlack(sta::MinMax::min());
  return -new_wns - 0.1f * current_step_;  // Example
}

bool EcoEnvironment::isTimingClosed()
{
  return sta_->worstSlack(sta::MinMax::min()) >= 0;
}

}  // namespace rl_eco
