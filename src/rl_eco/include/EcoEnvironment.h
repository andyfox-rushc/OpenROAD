// EcoEnvironment.h
#ifndef RL_ECO_ENVIRONMENT_H
#define RL_ECO_ENVIRONMENT_H

#include <vector>
#include <torch/torch.h>  // For tensors (observations/actions)
#include "odb/db.h"
#include "sta/Sta.hh"  // Assuming OpenROAD headers are included via CMake

namespace rl_eco {

struct EnvironmentConfig {
  std::string def_file;
  std::string lef_file;
  std::string liberty_file;
  std::vector<std::string> spare_cells;
  int max_steps = 50;
};

class EcoEnvironment {
public:
  EcoEnvironment(const EnvironmentConfig& config);
  ~EcoEnvironment();

  torch::Tensor reset();  // Returns initial observation tensor
  std::tuple<torch::Tensor, float, bool> step(int action);  // obs, reward, done

private:
  odb::dbDatabase* db_;
  sta::Sta* sta_;
  std::vector<std::string> spare_cells_;
  int current_step_;
  int max_steps_;

  torch::Tensor getObservation();  // e.g., tensor of slacks + spares
  void applyEco(int action);  // Decode and apply spare cell assignment
  float computeReward();
  bool isTimingClosed();
};

}  // namespace rl_eco

#endif
