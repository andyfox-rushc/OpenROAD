#pragma once

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include "rl_eco/EcoRLEnvironment.h"  // Include for QLearningConfig
#include "rl_eco/DQNAgent.h"  // Include for QLearningConfig

namespace utl {
class Logger;
}

namespace eco {

// Forward declarations
class EcoDesignManager;
class DQNAgent;
struct EcoAction; /* in EcoRLEnvironment.h */
struct QLearningConfig;

// Q-Learning Trainer for ECO
class EcoQLearningTrainer {
public:
  
    EcoQLearningTrainer(std::shared_ptr<EcoDesignManager> manager,
			const QLearningConfig& config,
			utl::Logger* logger);
    EcoQLearningTrainer(std::shared_ptr<EcoDesignManager> manager,
			const QLearningConfig& config,
			utl::Logger* logger,
			const char* filepath,
			size_t feature_size,
			size_t action_size
			);
    ~EcoQLearningTrainer() = default;
    // Agent access
    DQNAgent* getAgent() { return agent_.get(); }
    const DQNAgent* getAgent() const { return agent_.get(); }
    // Direct access to common agent operations
    void decayEpsilon() { agent_->decayEpsilon(); }
    const DQNAgent::TrainingStats& getStats() const { return agent_->getStats(); }

    
    // Training methods
    void train(size_t num_episodes);
    void trainEpisode();
    void applyInference();  
    double evaluate(size_t num_episodes = 10);
    double getBestReward() const { return best_episode_reward_; }
    
    // Results
  std::vector<std::shared_ptr<EcoAction> >  getBestActionSequence();

    
    // Model persistence
    void saveAgent(const std::string& filename) const;
    void loadAgent(const std::string& filename);
    void saveTrainingStats(const std::string& filename) const;
    
    // Reporting
    std::string generateTrainingReport() const;
    void printTrainingProgress(size_t episode);
  size_t max_action_size() {env_ -> max_action_size();}
  size_t state_size() {env_ -> state_size();}
  
private:
    std::shared_ptr<EcoDesignManager> manager_;
  std::unique_ptr<EcoRLEnvironment> env_;
    std::unique_ptr<DQNAgent> agent_;
    QLearningConfig config_;
    
  // Training state
  size_t total_steps_;
  double best_episode_reward_;
  std::vector<std::shared_ptr<EcoAction> > best_action_sequence_;
  utl::Logger* logger_;
  
  // Helper methods
  void recordEpisodeStats(double reward, size_t length);
};

// Utility functions for Q-Learning
namespace QLearningUtils {
    
    // Moving average for smoothing training curves
    std::vector<double> movingAverage(const std::vector<double>& data, size_t window_size);
    
    // Analyze action distribution
    std::unordered_map<std::string, size_t> analyzeActionDistribution(
        const std::vector<EcoAction>& actions);
    
    // Training checkpoints
    struct TrainingCheckpoint {
        size_t episode;
        double best_reward;
        std::string model_path;
        std::string timestamp;
    };
    
    void saveCheckpoint(const TrainingCheckpoint& checkpoint, const std::string& filename);
    TrainingCheckpoint loadCheckpoint(const std::string& filename);
}

} // namespace eco
