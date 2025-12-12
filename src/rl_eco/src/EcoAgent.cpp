// EcoAgent.cpp
#include "EcoAgent.h"

#include <vector>  // For trajectory storage

#include "EcoEnvironment.h"

namespace rl_eco {

EcoAgent::ActorCritic::ActorCritic(int obs_size, int act_size)
    : actor_fc1(obs_size, 64),
      actor_fc2(64, 64),
      actor_out(64, act_size),
      critic_fc1(obs_size, 64),
      critic_fc2(64, 64),
      critic_out(64, 1)
{
  register_module("actor_fc1", actor_fc1);
  // Register others similarly
}

std::tuple<torch::Tensor, torch::Tensor> EcoAgent::ActorCritic::forward(
    const torch::Tensor& obs)
{
  auto x = torch::relu(actor_fc1(obs));
  x = torch::relu(actor_fc2(x));
  auto action_logits = actor_out(x);

  auto v = torch::relu(critic_fc1(obs));
  v = torch::relu(critic_fc2(v));
  v = critic_out(v);

  return {action_logits, v};
}

EcoAgent::EcoAgent(int obs_size, int act_size)
    : ac_model_(std::make_shared<ActorCritic>(obs_size, act_size)),
      optimizer_(ac_model_->parameters(), torch::optim::AdamOptions(3e-4))
{
}

void EcoAgent::train(EcoEnvironment& env, int total_timesteps)
{
  // Simplified PPO loop: Collect trajectories, compute advantages, update
  for (int t = 0; t < total_timesteps; ++t) {
    auto obs = env.reset();
    bool done = false;
    std::vector<torch::Tensor> obs_list, act_list, logp_list, rew_list,
        val_list;

    while (!done) {
      auto [logits, value] = ac_model_->forward(obs);
      auto probs = torch::softmax(logits, 0);
      int action = torch::multinomial(probs, 1).item<int>();  // Sample action
      auto logp = torch::log(probs[action]);

      auto [next_obs, reward, done] = env.step(action);

      // Store
      obs_list.push_back(obs);
      act_list.push_back(torch::tensor({action}));
      logp_list.push_back(logp);
      rew_list.push_back(torch::tensor({reward}));
      val_list.push_back(value);

      obs = next_obs;
    }

    // Compute advantages & returns (simplified; use GAE for better)
    // ... (implement advantage estimation here)

    // PPO update: Compute clipped loss, backprop
    optimizer_.zero_grad();
    // loss = ... (surrogate objective + value loss + entropy)
    // loss.backward();
    // optimizer_.step();
  }
}

int EcoAgent::predict(const torch::Tensor& obs, bool det)
{
  torch::NoGradGuard no_grad;
  auto [logits, _] = ac_model_->forward(obs);
  if (det) {
    return torch::argmax(logits).item<int>();
  }
  return torch::multinomial(torch::softmax(logits, 0), 1).item<int>();
}

// save/load as before
void EcoAgent::save(const std::string& path)
{
  torch::save(ac_model_, path);
}
EcoAgent EcoAgent::load(const std::string& path)
{
  EcoAgent agent(0, 0);
  torch::load(agent.ac_model_, path);
  return agent;
}

}  // namespace rl_eco
