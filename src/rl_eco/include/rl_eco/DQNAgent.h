#pragma once

#include <memory>
#include <vector>
#include <string>
#include "rl_eco/EcoNeuralNetwork.h" //for replayBuffer and Experience

#include <random>
#include <deque>

namespace eco {

// Forward declarations
class EcoRLEnvironment;
class NeuralNetwork;
class ReplayBuffer;


// Q-Learning configuration
struct QLearningConfig {
    // Network architecture
  std::vector<size_t> hidden_layers = {256,256,256};//{128, 64, 32};
  size_t default_action_size = 500;
    // Learning parameters
    double learning_rate = 0.001;
    double gamma = 0.99;  // Discount factor
    double epsilon_start = 1.0;
    double epsilon_end = 0.01;
    double epsilon_decay = 0.995;
    
    // Training parameters
    size_t batch_size = 32;
    size_t replay_buffer_size = 10000;
    size_t target_update_frequency = 100;
    size_t update_frequency = 4;
    size_t max_steps_per_episode = 1000;
    size_t max_episodes = 1000;
    
    // DQN variants
    bool use_double_dqn = true;
    bool use_prioritized_replay = false;
    
    // Reward weights
    double timing_weight = 1.0;
    double power_weight = 0.5;
    double wirelength_weight = 0.3;
    double completion_bonus = 100.0;
    double invalid_action_penalty = -10.0;
  double congestion_weight = 0.3;
};

// Deep Q-Network Agent
class DQNAgent {
public:
    DQNAgent(size_t state_size, size_t action_size, const QLearningConfig& config);
    ~DQNAgent() = default;

  //debug
  std::vector<double> getQValues(const std::vector<double>& state) {
    auto q_values = q_network_->forward(state);
    return q_values;
  }
    // Core methods
    size_t selectAction(const std::vector<double>& state, 
                       const std::vector<size_t>& valid_actions,
                       bool training = true);
    void remember(const Experience& exp);
    double train();
    
    // Epsilon management
    double getEpsilon() const { return epsilon_; }
    void setEpsilon(double eps) { epsilon_ = eps; }
    void decayEpsilon() { 
        epsilon_ = std::max(config_.epsilon_end, epsilon_ * config_.epsilon_decay); 
    }
    
    // Model management
    void updateTargetNetwork();
    void saveModel(const std::string& filename) const;
    void loadModel(const std::string& filename);
    
    // Statistics
    struct TrainingStats {
        std::vector<double> episode_rewards;
        std::vector<double> episode_lengths;
        std::vector<double> losses;
        std::vector<double> q_values;
        std::vector<double> epsilons;
    };
    

    TrainingStats& getStats() { return stats_; }  // Non-const version for EcoQLearningTrainer
    void setInferenceMode(bool mode) {inference_mode_i = mode;}
    bool isInferenceMode() const {return inference_mode_i;}
    
private:
    size_t state_size_;
    size_t action_size_;
    QLearningConfig config_;
    
    std::unique_ptr<NeuralNetwork> q_network_;
    std::unique_ptr<NeuralNetwork> target_network_;
    std::unique_ptr<ReplayBuffer> replay_buffer_;
    
    double epsilon_;
    size_t train_step_;
    TrainingStats stats_;
    bool inference_mode_i ;
    // Random number generation
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> uniform_dist_;
    
    // Helper methods
    std::vector<double> computeQValues(const std::vector<double>& state);
    void copyNetworkWeights(NeuralNetwork* from, NeuralNetwork* to);
};

} // namespace eco
