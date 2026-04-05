#include "ord/OpenRoad.hh"
#include "rl_eco/EcoTypes.h"
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

  //constructor for inference mode
  EcoQLearningTrainer::EcoQLearningTrainer(std::shared_ptr<EcoDesignManager> manager,
					   const QLearningConfig& config,
					   utl::Logger* logger,
					   const char* filepath,
					   size_t feature_size,
					   size_t action_size)
    : manager_(manager), 
      config_(config),
      total_steps_(0),
      best_episode_reward_(-std::numeric_limits<double>::infinity()),
      logger_(logger) {

    rsz::Resizer* resizer = ord::OpenRoad::openRoad() -> getResizer();
    env_ = std::make_unique<EcoRLEnvironment>(manager, resizer);
    agent_ = std::make_unique<DQNAgent>(feature_size,action_size, config);
    agent_ -> loadModel(filepath);
    agent_ -> setInferenceMode(true);
    printf("Making inference agent for state size %d and Action size %d\n",
	   feature_size,
	   action_size);
  }


  //constructor for learning mode
EcoQLearningTrainer::EcoQLearningTrainer(std::shared_ptr<EcoDesignManager> manager,
					 const QLearningConfig& config,
					 utl::Logger* logger)
    : manager_(manager), 
      config_(config),
      total_steps_(0),
      best_episode_reward_(-std::numeric_limits<double>::infinity()),
      logger_(logger) {

  //Pass in resizer from openroad (note this should be cleaned up
  //so that eco initializes its own copy).
  rsz::Resizer* resizer = ord::OpenRoad::openRoad() -> getResizer();
  
  env_ = std::make_unique<EcoRLEnvironment>(manager, resizer);
    
    // Create agent
    size_t state_size = env_->getStateSize();
    size_t action_size = env_->getActionSize();
    //Note that until we start training we don't have any actions
    //so we default the action size
    if (action_size == 0){
      action_size = config.default_action_size;
    }
    agent_ = std::make_unique<DQNAgent>(state_size, action_size, config);
}

void EcoQLearningTrainer::train(size_t num_episodes) {
    for (size_t episode = 0; episode < num_episodes; ++episode) {
        trainEpisode();
        
        // Decay epsilon
        agent_->decayEpsilon();
        
        // Print progress after every episode for now
	//        if (episode % 10 == 0) {
            printTrainingProgress(episode);
	    //        }
        
        // Save checkpoint
        if (episode % 100 == 0 && episode > 0) {
            std::string checkpoint_file = "checkpoint_episode_" + 
                                         std::to_string(episode) + ".ckpt";
            saveAgent(checkpoint_file);
        }
    }
}

  void EcoQLearningTrainer::applyInference(){
  env_->reset();
  auto state = env_ -> getCurrentState();
  bool done = false;
  int steps=0;
  agent_ -> setInferenceMode(true);
  
  while (!done){
        // Get current state as feature vector
      auto state_features = state.toVector();
      env_ -> state_size(state_features.size() > env_ -> state_size()?
			 state_features.size():
			 env_ -> state_size());
      
      // Get valid actions for this episode.
      // Side effect is to construct means of indexing actions
      // (populates environment action2index, index2action)
      //
      auto valid_actions_enum = env_->getValidActions(state);

      logger_->info(utl::ECO, 998, "Inference Moves {}:  TNS {} WNS {}",
		    steps,
		    state.tns,
		    state.wns);

      // Get Q-values for all actions (before selection)

      if (state_features.size() < env_ -> getStateSize()){
	for (size_t s = state_features.size();
	     s != env_ -> getStateSize();
	     s++){
	  state_features.push_back(0.0);
	}
      }
      
      auto q_values = agent_->getQValues(state_features);
      
      printf("Number of q values %d when action count is %d\n",
	     q_values.size(),
	     valid_actions_enum.size()
	     );


      if (q_values.size() == 0){
	printf("Why no q values ???\n");
	exit(0);
      }
      
      //debug
      logger_->info(utl::ECO, 999, "Q-values for valid actions:");
      for (size_t i = 0; i < valid_actions_enum.size(); ++i) {
	//action is std::shared_ptr<EcoAction>
	const auto action = valid_actions_enum[i];
	size_t idx = env_ -> getIndexFromAction(action);
	if (idx < q_values.size()) {
	  logger_->info(utl::ECO, 999, "  {} : Q = {:.10f}", 
			action -> toString(),
			q_values[idx]);
	}
      }
      
      std::vector<size_t> valid_actions;
      for (const auto& action : valid_actions_enum) {
	//action is std::shared_ptr<EcoAction>
	valid_actions.push_back(env_ -> getIndexFromAction(action));
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
      std::shared_ptr<EcoAction> action = env_ -> getActionFromIndex(action_index);

      bool done=false;
      // Take action
      env_ -> resizer_->journalBegin();      
      EcoDesignManager::MoveResult result = env_ -> executeMove(action);      
      if (result.timing_improvement < 0.0){
	printf("Things got worse ! for tns by %.10f \n", result.timing_improvement);	
	env_ -> resizer_->journalRestore();
	done=true;
      }
      else{
	printf("Things getting better in tns by %.10f!\n",result.timing_improvement);
      }
      env_ -> resizer_ -> journalEnd();
      
      auto next_state = env_ -> getCurrentState();
      if (env_ -> isEpisodeDone()){
	done = true;
      }
      
      if (done){
	printf("inference is done\n");
	break;
      }
      // Update state
      state = next_state;
      printf("Updated state\n");
      steps++;
      total_steps_++;
    }
  }
  


  
void EcoQLearningTrainer::trainEpisode() {
  env_->reset();
  auto state = env_ -> getCurrentState();
  double episode_reward = 0.0;
  size_t steps = 0;
  std::vector<std::shared_ptr<EcoAction> > episode_actions;
  int zero_action_count=0;
  
    while (steps < config_.max_steps_per_episode) {
        // Get current state as feature vector
      auto state_features = state.toVector();

      // Get valid actions for this episode.
      // Side effect is to construct means of indexing actions
      // (populates environment action2index, index2action)
      //
      printf("Getting valid  actions \n");
      auto valid_actions_enum = env_->getValidActions(state);
      printf("Number of valid actions %d\n", valid_actions_enum.size());
      if (valid_actions_enum.size() == 0){
	if (zero_action_count >=2){
	  return;
	}
	zero_action_count = zero_action_count+1;
      }
      else {
	zero_action_count = 0;
      }
      
      logger_->info(utl::ECO, 998, "Step {}: Moves attempted {}: Moves accepted {} TNS {} WNS {}",
		    steps,
		    state.moves_attempted,
		    state.moves_accepted,
		    state.tns,
		    state.wns);

      // Get Q-values for all actions (before selection)

      printf("State features size =%d state_size = %d\n",
	     state_features.size(),
	     env_ -> getStateSize());
      if (state_features.size() < env_ -> getStateSize()){
	for (size_t s = state_features.size();
	     s != env_ -> getStateSize();
	     s++){
	  state_features.push_back(0.0);
	}
      }
      
      auto q_values = agent_->getQValues(state_features);
      
      printf("Number of q values %d when action count is %d\n",
	     q_values.size(),
	     valid_actions_enum.size()
	     );


      assert (q_values.size() != 0);
      
      //debug
      logger_->info(utl::ECO, 999, "Q-values for valid actions:");
      for (size_t i = 0; i < valid_actions_enum.size(); ++i) {
	//action is std::shared_ptr<EcoAction>
	const auto action = valid_actions_enum[i];
	size_t idx = env_ -> getIndexFromAction(action);
	if (idx < q_values.size()) {
	  logger_->info(utl::ECO, 999, "  {} : Q = {:.10f}", 
			action -> toString(),
			q_values[idx]);
	}
      }
      
      std::vector<size_t> valid_actions;
      for (const auto& action : valid_actions_enum) {
	//action is std::shared_ptr<EcoAction>
	valid_actions.push_back(env_ -> getIndexFromAction(action));
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
      std::shared_ptr<EcoAction> action = env_ -> getActionFromIndex(action_index);
        
      // Take action
      double reward = env_->step(action);
      auto next_state = env_ -> getCurrentState();
      bool done = env_->isEpisodeDone();

      //record the action
      episode_actions.push_back(action);
      // Get next state features
      auto next_state_features = next_state.toVector();
        
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

	
        if (done) {
            break;
        }
    }
    
    // Record episode statistics
    recordEpisodeStats(episode_reward, steps);
    
    // Update best episode
    if (episode_reward > best_episode_reward_) {
        best_episode_reward_ = episode_reward;
        best_action_sequence_ = episode_actions;
    }
}

double EcoQLearningTrainer::evaluate(size_t num_episodes) {
    double total_reward = 0.0;
    
    for (size_t episode = 0; episode < num_episodes; ++episode) {
      env_->reset();
      auto state = env_ -> getCurrentState();
        double episode_reward = 0.0;
        size_t steps = 0;
        
        while (steps < config_.max_steps_per_episode) {
	  auto state_features = state.toVector();
            auto valid_actions_enum = env_->getValidActions(state);
            std::vector<size_t> valid_actions;
            for (const auto& action : valid_actions_enum) {
	      valid_actions.push_back(env_ -> getIndexFromAction(action));
            }
            
            if (valid_actions.empty()) {
                break;
            }
            
            // Select action without exploration
            size_t action_index = agent_->selectAction(state_features, valid_actions, false);
	    std::shared_ptr<EcoAction> action = env_ -> getActionFromIndex(action_index);
            
	    double reward = env_->step(action);
            auto state = env_ -> getCurrentState();
            episode_reward += reward;
            steps++;
            
            if (env_->isEpisodeDone()) {
                break;
            }
        }
        
        total_reward += episode_reward;
    }
    
    return total_reward / num_episodes;
}

  std::vector<std::shared_ptr<EcoAction> > EcoQLearningTrainer::getBestActionSequence() {
    return best_action_sequence_;
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
    
    report << "\nBest Action Sequence (" << best_action_sequence_.size() << " actions):\n";
    for (size_t i = 0; i < best_action_sequence_.size(); ++i) {
      report << "  " << i + 1 << ". " << best_action_sequence_[i]->toString() << "\n";
      if (best_action_sequence_[i]  -> type == eco::EcoAction::ActionType::RESIZE){
	printf(" Instance %s  -> Instance %s (%s) Improvement %.10f\n",
	       best_action_sequence_[i]  -> resize -> instance_name.c_str(),
	       best_action_sequence_[i]  -> resize -> spare_instance.c_str(),
	       best_action_sequence_[i]  -> resize -> new_master.c_str(),
	       best_action_sequence_[i]  -> resize -> predicted_improvement);
      }
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
								  const std::vector<std::shared_ptr<EcoAction> > & actions) {
    std::unordered_map<std::string, size_t> distribution;
    
    for (const auto& action : actions) {
        std::string action_type;
        switch (action -> type) {
	case EcoAction::ActionType::RESIZE:
	  action_type = "RESIZE";
	  break;
	case EcoAction::ActionType::REBUFFER:
	  action_type = "REBUFFER";
	  break;
	default:
	  break;
        }
        distribution[action_type]++;
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
