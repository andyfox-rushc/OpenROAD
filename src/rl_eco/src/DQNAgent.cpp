
#include "rl_eco/EcoTypes.h"
#include "rl_eco/DQNAgent.h"
#include "rl_eco/EcoNeuralNetwork.h"
#include "rl_eco/EcoRLEnvironment.h"
#include "rl_eco/DQNAgent.h"


#include <algorithm>
#include <numeric>
#include <cstdio>

namespace eco {

DQNAgent::DQNAgent(size_t state_size, size_t action_size, const QLearningConfig& config)
    : state_size_(state_size), 
      action_size_(action_size), 
      config_(config),
      epsilon_(config.epsilon_start),
      train_step_(0),
      inference_mode_i(false),      
      generator_(std::random_device{}()),
      uniform_dist_(0.0, 1.0)
       {
    
    // Build network architecture
	 //	 printf("DQNAgent layers:\n");
    std::vector<size_t> layer_sizes;
    layer_sizes.push_back(state_size);
    int ix=0;
    for (size_t hidden_size : config.hidden_layers) {
      //      printf("DQNAgent Layer %d size %d\n",ix, hidden_size);
      layer_sizes.push_back(hidden_size);
      ix++;
    }
    //    printf("Building last layer of size %d\n",action_size);
    layer_sizes.push_back(action_size);
    
    // Create Q-network and target network
    q_network_ = std::make_unique<NeuralNetwork>(layer_sizes);
    target_network_ = std::make_unique<NeuralNetwork>(layer_sizes);
    
    // Initialize target network with same weights
    copyNetworkWeights(q_network_.get(), target_network_.get());
    
    // Create replay buffer
    replay_buffer_ = std::make_unique<ReplayBuffer>(config.replay_buffer_size);
}


  size_t DQNAgent::selectAction(const std::vector<double>& state, 
                             const std::vector<size_t>& valid_actions,
                             bool training) {
    // In inference mode, always exploit (no exploration)
    if (inference_mode_i || (!training && std::rand() / double(RAND_MAX) >= epsilon_)) {
      // Fast path for inference - direct forward pass
      //
      //The q values come from the weights learned during training.
      //
      std::vector<double> state_copy = state;
        auto q_values = q_network_->forward(state_copy);
        // Find the best valid action
        double max_q = -std::numeric_limits<double>::infinity();
        size_t best_action = valid_actions[0];
        for (size_t action : valid_actions) {
            if (q_values[action] > max_q) {
                max_q = q_values[action];
                best_action = action;
            }
        }
	printf("Inference, best action %d\n", best_action);
        return best_action;
    }
    printf("Exploration: Valid actions size %d !\n",valid_actions.size());
    // Exploration: random valid action
    printf("Value of std::rand() : %d\n", std::rand());
    int random_ix = std::rand() % valid_actions.size();
    printf("Exploration. Random action: %d\n",random_ix);
    return valid_actions[random_ix];
}


void DQNAgent::remember(const Experience& exp) {
  if (inference_mode_i)
    return; //skip memory storage in inference mode
  replay_buffer_->add(exp);
}

  /*
    / This is the core Bellman update happening here:
    // Q(s,a) ← Q(s,a) + α[r + γ max_a' Q(s',a') - Q(s,a)]
    //
    // The code implements this as:
    // 1. Q(s,a) = current_q_values[action]
    // 2. target = r + γ max_a' Q(s',a')  
    // 3. Network update moves Q(s,a) toward target
    */
  
double DQNAgent::train() {
    if (!replay_buffer_->canSample(config_.batch_size)) {
        return 0.0;
    }
    
    auto batch = replay_buffer_->sample(config_.batch_size);
    double total_loss = 0.0;
    
    for (const auto& exp : batch) {
      // Compute current Q-value
      // This is Q(s,a) - our current estimate, not the "true" value
      std::vector<double> state_mutable = exp.state;
      std::vector<double> current_q_values = q_network_->forward(state_mutable);

      // 2. BOOTSTRAP TARGET CALCULATION
      // Compute target Q-value
        double target_q_value;
        if (exp.done) {
	  // Terminal state: no future value
            target_q_value = exp.reward;
        } else {
	  // Non-terminal: bootstrap using our ESTIMATE of future value
	  std::vector<double> muteable_values = exp.next_state;
	  auto next_q_values = target_network_->forward(muteable_values);
            if (config_.use_double_dqn) {
                // Double DQN: use online network to select action, target network to evaluate
                auto next_q_values_online = q_network_->forward(next_q_values);
                size_t best_action = std::distance(
                    next_q_values_online.begin(),
                    std::max_element(next_q_values_online.begin(), next_q_values_online.end())
                );
		// This is the key bootstrapping step!
		// We use our CURRENT ESTIMATE of future Q-values
		// Not the true future values (which we don't know)
                target_q_value = exp.reward + config_.gamma * next_q_values[best_action];
            } else {
                // Standard DQN
                target_q_value = exp.reward + config_.gamma * 
                    *std::max_element(next_q_values.begin(), next_q_values.end());
            }
        }
        
        // Create target vector
	// 3. INCREMENTAL UPDATE
	// Move our estimate toward the bootstrap target
        std::vector<double> target = current_q_values;
        target[exp.action] = target_q_value;
        
        // Train network
	// Neural network does: Q(s,a) += learning_rate * (target - Q(s,a))
	std::vector<double> muteable_values = exp.next_state;
	q_network_->train(muteable_values, target, config_.learning_rate);
        
        // Calculate loss for statistics
        double loss = q_network_->computeLoss(current_q_values, target);
        total_loss += loss;
    }
    
    train_step_++;
    
    // Update target network periodically
    if (train_step_ % config_.target_update_frequency == 0) {
        updateTargetNetwork();
    }
    
    double avg_loss = total_loss / batch.size();
    stats_.losses.push_back(avg_loss);
    
    return avg_loss;
}

void DQNAgent::updateTargetNetwork() {
    copyNetworkWeights(q_network_.get(), target_network_.get());
}

void DQNAgent::copyNetworkWeights(NeuralNetwork* from, NeuralNetwork* to) {
    // This is a simplified version - in practice, you'd need access to internal weights
    // For now, we'll save and load through files (not efficient but works)
    const std::string temp_file = ".temp_network_weights.bin";
    from->save(temp_file);
    //    to->load(temp_file);
    std::remove(temp_file.c_str());

    size_t num_layers_from = from -> getNumLayers();
    size_t num_layers_to = to -> getNumLayers();
    assert(num_layers_from == num_layers_to);

    to -> copyWeightsFrom(*from);
}

std::vector<double> DQNAgent::computeQValues(const std::vector<double>& state) {
  std::vector<double> copy_of_state = state;
    return q_network_->forward(copy_of_state);
}

void DQNAgent::saveModel(const std::string& filename) const {
    q_network_->save(filename);
}

void DQNAgent::loadModel(const std::string& filename) {
    q_network_->load(filename);
    updateTargetNetwork();
}

} // namespace eco
