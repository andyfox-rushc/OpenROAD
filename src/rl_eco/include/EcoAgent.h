// EcoAgent.h
#ifndef RL_ECO_AGENT_H
#define RL_ECO_AGENT_H

#include <torch/torch.h>
// No <torchrl/...> needed

namespace rl_eco {

  class EcoEnvironment;
  
class EcoAgent {
public:
  EcoAgent(int observation_size, int action_size);
  void train(EcoEnvironment& env, int total_timesteps);
  int predict(const torch::Tensor& obs, bool deterministic = true);
  void save(const std::string& path);
  static EcoAgent load(const std::string& path);

private:
  struct ActorCritic : torch::nn::Module {
    torch::nn::Linear actor_fc1, actor_fc2, actor_out;
    torch::nn::Linear critic_fc1, critic_fc2, critic_out;
    ActorCritic(int obs_size, int act_size);
    std::tuple<torch::Tensor, torch::Tensor> forward(const torch::Tensor& obs);
  };

  std::shared_ptr<ActorCritic> ac_model_;
  torch::optim::Adam optimizer_;
  // Hyperparams (add more: clip_param=0.2, etc.)
};

}  // namespace rl_eco

#endif
