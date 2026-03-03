#include "rl_eco/EcoQLearningTrainer.h"
#include "rl_eco/EcoNeuralNetwork.h"
#include "rl_eco/DQNAgent.h"
#include "rl_eco/EcoRLEnvironment.h"
#include "rl_eco/EcoDesignManager.h"
#include "rl_eco/DQNAgent.h"  // Include for QLearningConfig
#include "utl/Logger.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <ctime>
#include <sstream>
#include <limits>

namespace eco {

EcoQLearningTrainer::EcoQLearningTrainer(std::shared_ptr<EcoDesignManager> manager,
					 const QLearningConfig& config,
					 utl::Logger* logger)
    : manager_(manager), 
      config_(config),
      total_steps_(0),
      best_episode_reward_(-std::numeric_limits<double>::infinity()),
      logger_(logger) {
    
    env_ = std::make_unique<EcoRLEnvironment>(manager, manager->logger());
    
    // Create agent
    size_t state_size = env_->getStateSize();
    size_t action_size = env_->getActionSize();
    agent_ = std::make_unique<DQNAgent>(state_size, action_size, config);
}

void EcoQLearningTrainer::train(size_t num_episodes) {
    for (size_t episode = 0; episode < num_episodes; ++episode) {
        trainEpisode();
        
        // Decay epsilon
        agent_->decayEpsilon();
        
        // Print progress
        if (episode % 10 == 0) {
            printTrainingProgress(episode);
        }
        
        // Save checkpoint
        if (episode % 100 == 0 && episode > 0) {
            std::string checkpoint_file = "checkpoint_episode_" + 
                                         std::to_string(episode) + ".ckpt";
            saveAgent(checkpoint_file);
        }
    }
}

void EcoQLearningTrainer::trainEpisode() {
    auto state = env_->reset();
    double episode_reward = 0.0;
    size_t steps = 0;
    std::vector<EcoMove> episode_moves;
    
    while (steps < config_.max_steps_per_episode) {
        // Get current state as feature vector
        auto state_features = state.toFeatureVectorEnhanced();
        
        // Get valid actions
        auto valid_actions_enum = env_->getValidActions(state);

	logger_->info(utl::ECO, 998, "Step {}: State - {} changes left, {} spares, type: {}",
                      steps, state.num_unimplemented_changes, 
                      state.num_available_spare_cells,
                      state.current_change_type);
        // Get Q-values for all actions (before selection)
        auto q_values = agent_->getQValues(state_features);
        
        // DEBUG: Log Q-values for valid actions
        logger_->info(utl::ECO, 999, "Q-values for valid actions:");
        for (size_t i = 0; i < valid_actions_enum.size(); ++i) {
            const auto& action = valid_actions_enum[i];
            size_t idx = action.toIndex();
            if (idx < q_values.size()) {
                logger_->info(utl::ECO, 999, "  {} : Q = {:.2f}", 
                             action.toString(), q_values[idx]);
            }
        }
        std::vector<size_t> valid_actions;
        for (const auto& action : valid_actions_enum) {
            valid_actions.push_back(action.toIndex());
        }
        
        if (valid_actions.empty()) {
            break;
        }
        
        // Select action
        size_t action_index = agent_->selectAction(state_features, valid_actions, true);

	if (action_index >= env_ -> getActionSize()){
	  std::cerr << "Warning: Invalid action index " << action_index << " >= " <<
	    env_ -> getActionSize() << std::endl;
	  continue;
	}
	
        // Convert index back to action
        EcoAction action = EcoAction::fromIndex(action_index);
        
        // Take action
        auto [next_state, reward] = env_->step(action);
        bool done = env_->isDone();
        
        // Get next state features
        auto next_state_features = next_state.toFeatureVectorEnhanced();
        
        // Store experience
        Experience exp{state_features, action_index, reward, next_state_features, done};
        agent_->remember(exp);
        
        // Train agent
        if (total_steps_ % config_.update_frequency == 0) {
            agent_->train();
        }
        
        // Update state
        state = next_state;
        episode_reward += reward;
        steps++;
        total_steps_++;
        
        // Record move
        auto possible_moves = manager_->generatePossibleMoves();
        if (!possible_moves.empty()) {
            // Find corresponding move
            for (const auto& move : possible_moves) {
                // Match action to move (simplified - would need proper mapping)
                if ((action.type == EcoAction::USE_SPARE_CELL && move.type == EcoMove::USE_SPARE_CELL) ||
                    (action.type == EcoAction::SKIP_CHANGE && move.type == EcoMove::SKIP)) {
                    episode_moves.push_back(move);
                    break;
                }
            }
        }
        
        if (done) {
            break;
        }
    }
    
    // Record episode statistics
    recordEpisodeStats(episode_reward, steps);
    
    // Update best episode
    if (episode_reward > best_episode_reward_) {
        best_episode_reward_ = episode_reward;
        best_move_sequence_ = episode_moves;
    }
}

double EcoQLearningTrainer::evaluate(size_t num_episodes) {
    double total_reward = 0.0;
    
    for (size_t episode = 0; episode < num_episodes; ++episode) {
        auto state = env_->reset();
        double episode_reward = 0.0;
        size_t steps = 0;
        
        while (steps < config_.max_steps_per_episode) {
            auto state_features = state.toFeatureVectorEnhanced();
            auto valid_actions_enum = env_->getValidActions(state);
            std::vector<size_t> valid_actions;
            for (const auto& action : valid_actions_enum) {
                valid_actions.push_back(action.toIndex());
            }
            
            if (valid_actions.empty()) {
                break;
            }
            
            // Select action without exploration
            size_t action_index = agent_->selectAction(state_features, valid_actions, false);
            EcoAction action = EcoAction::fromIndex(action_index);
            
            auto [next_state, reward] = env_->step(action);
            
            state = next_state;
            episode_reward += reward;
            steps++;
            
            if (env_->isDone()) {
                break;
            }
        }
        
        total_reward += episode_reward;
    }
    
    return total_reward / num_episodes;
}

std::vector<EcoMove> EcoQLearningTrainer::getBestMoveSequence() {
    return best_move_sequence_;
}

void EcoQLearningTrainer::printTrainingProgress(size_t episode) {
    const auto& stats = agent_->getStats();
    
    std::cout << "Episode " << episode << "/" << config_.max_episodes << std::endl;
    std::cout << "  Epsilon: " << agent_->getEpsilon() << std::endl;
    
    if (!stats.episode_rewards.empty()) {
        size_t window = std::min(size_t(100), stats.episode_rewards.size());
        double avg_reward = 0.0;
        for (size_t i = stats.episode_rewards.size() - window; 
             i < stats.episode_rewards.size(); ++i) {
            avg_reward += stats.episode_rewards[i];
        }
        avg_reward /= window;
        
        std::cout << "  Avg Reward (last " << window << " episodes): " 
                  << avg_reward << std::endl;
        std::cout << "  Best Episode Reward: " << best_episode_reward_ << std::endl;
    }
    
    if (!stats.losses.empty()) {
        double recent_loss = stats.losses.back();
        std::cout << "  Recent Loss: " << recent_loss << std::endl;
    }
    
    std::cout << "  Total Steps: " << total_steps_ << std::endl;
    std::cout << std::endl;
}

void EcoQLearningTrainer::recordEpisodeStats(double reward, size_t length) {
    auto& stats = agent_->getStats();
    stats.episode_rewards.push_back(reward);
    stats.episode_lengths.push_back(static_cast<double>(length));
    stats.epsilons.push_back(agent_->getEpsilon());
}

std::string EcoQLearningTrainer::generateTrainingReport() const {
    std::stringstream report;
    const auto& stats = agent_->getStats();
    
    report << "ECO Q-Learning Training Report\n";
    report << "==============================\n\n";
    
    report << "Configuration:\n";
    report << "  Hidden Layers: ";
    for (size_t size : config_.hidden_layers) {
        report << size << " ";
    }
    report << "\n";
    report << "  Learning Rate: " << config_.learning_rate << "\n";
    report << "  Gamma: " << config_.gamma << "\n";
    report << "  Batch Size: " << config_.batch_size << "\n";
    report << "  Replay Buffer Size: " << config_.replay_buffer_size << "\n\n";
    
    report << "Training Results:\n";
    report << "  Total Episodes: " << stats.episode_rewards.size() << "\n";
    report << "  Total Steps: " << total_steps_ << "\n";
    report << "  Best Episode Reward: " << best_episode_reward_ << "\n";
    
    if (!stats.episode_rewards.empty()) {
        double avg_reward = std::accumulate(stats.episode_rewards.begin(), 
                                          stats.episode_rewards.end(), 0.0) / 
                           stats.episode_rewards.size();
        report << "  Average Episode Reward: " << avg_reward << "\n";
    }
    
    report << "\nBest Move Sequence (" << best_move_sequence_.size() << " moves):\n";
    for (size_t i = 0; i < best_move_sequence_.size(); ++i) {
        report << "  " << i + 1 << ". " << best_move_sequence_[i].getDescription() << "\n";
    }
    
    return report.str();
}

void EcoQLearningTrainer::saveAgent(const std::string& filename) const {
    agent_->saveModel(filename);
}

void EcoQLearningTrainer::loadAgent(const std::string& filename) {
    agent_->loadModel(filename);
}

void EcoQLearningTrainer::saveTrainingStats(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    
    const auto& stats = agent_->getStats();
    
    file << "episode,reward,length,epsilon,loss\n";
    
    size_t max_size = stats.episode_rewards.size();
    for (size_t i = 0; i < max_size; ++i) {
        file << i << ",";
        file << stats.episode_rewards[i] << ",";
        file << stats.episode_lengths[i] << ",";
        file << (i < stats.epsilons.size() ? stats.epsilons[i] : 0.0) << ",";
        file << (i < stats.losses.size() ? stats.losses[i] : 0.0) << "\n";
    }
}

// QLearningUtils implementation
namespace QLearningUtils {

std::vector<double> movingAverage(const std::vector<double>& data, size_t window_size) {
    std::vector<double> result;
    if (data.size() < window_size) {
        return result;
    }
    
    for (size_t i = window_size - 1; i < data.size(); ++i) {
        double sum = 0.0;
        for (size_t j = 0; j < window_size; ++j) {
            sum += data[i - j];
        }
        result.push_back(sum / window_size);
    }
    
    return result;
}

std::unordered_map<std::string, size_t> analyzeActionDistribution(
    const std::vector<EcoMove>& moves) {
    
    std::unordered_map<std::string, size_t> distribution;
    
    for (const auto& move : moves) {
        std::string move_type;
        switch (move.type) {
            case EcoMove::MoveType::USE_SPARE_CELL:
                move_type = "USE_SPARE_CELL";
                break;
            case EcoMove::MoveType::CONNECT_NET:
                move_type = "CONNECT_NET";
                break;
            case EcoMove::MoveType::DISCONNECT_NET:
                move_type = "DISCONNECT_NET";
                break;
            case EcoMove::MoveType::CREATE_NET:
                move_type = "CREATE_NET";
                break;
            case EcoMove::MoveType::BUFFER_NET:
                move_type = "BUFFER_NET";
                break;
            case EcoMove::MoveType::SKIP:
                move_type = "SKIP";
                break;
        }
        distribution[move_type]++;
    }
    
    return distribution;
}

void saveCheckpoint(const TrainingCheckpoint& checkpoint, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) return;
    
    // Save checkpoint data
    file.write(reinterpret_cast<const char*>(&checkpoint.episode), sizeof(checkpoint.episode));
    file.write(reinterpret_cast<const char*>(&checkpoint.best_reward), sizeof(checkpoint.best_reward));
    
    // Save timestamp
    size_t timestamp_size = checkpoint.timestamp.size();
    file.write(reinterpret_cast<const char*>(&timestamp_size), sizeof(timestamp_size));
    file.write(checkpoint.timestamp.c_str(), timestamp_size);
    
    // Save model path
    size_t path_size = checkpoint.model_path.size();
    file.write(reinterpret_cast<const char*>(&path_size), sizeof(path_size));
    file.write(checkpoint.model_path.c_str(), path_size);
}

TrainingCheckpoint loadCheckpoint(const std::string& filename) {
    TrainingCheckpoint checkpoint;
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) return checkpoint;
    
    // Load checkpoint data
    file.read(reinterpret_cast<char*>(&checkpoint.episode), sizeof(checkpoint.episode));
    file.read(reinterpret_cast<char*>(&checkpoint.best_reward), sizeof(checkpoint.best_reward));
    
    // Load timestamp
    size_t timestamp_size;
    file.read(reinterpret_cast<char*>(&timestamp_size), sizeof(timestamp_size));
    checkpoint.timestamp.resize(timestamp_size);
    file.read(&checkpoint.timestamp[0], timestamp_size);
    
    // Load model path
    size_t path_size;
    file.read(reinterpret_cast<char*>(&path_size), sizeof(path_size));
    checkpoint.model_path.resize(path_size);
    file.read(&checkpoint.model_path[0], path_size);
    
    return checkpoint;
}

}  // namespace QLearningUtils

}  // namespace eco
