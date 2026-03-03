#include "odb/db.h"
#include "rl_eco/RlEco.h"
#include "rl_eco/EcoDesignManager.h"
#include "rl_eco/EcoQLearningTrainer.h"
#include "rl_eco/EcoRLEnvironment.h"
#include "rl_eco/EcoChangeSet.h"

#include "ord/OpenRoad.hh"

#include "sta/Sta.hh"
#include "utl/Logger.h"

#include <filesystem>
#include <chrono>
#include <cstring>
#include <random>
#include <algorithm>
#include <tcl.h>

using namespace ord;
namespace eco {

RlEco::RlEco() = default;
RlEco::~RlEco() = default;

  
void RlEco::init(OpenRoad* openroad)
{
 if (!openroad) {
        throw std::runtime_error("Null OpenRoad pointer provided to RlEco::init");
    }
     // Store the openroad pointer
    openroad_ = openroad;
    
    // Get the logger from openroad
    logger_ = openroad->getLogger();
    if (!logger_) {
        throw std::runtime_error("Failed to get logger from OpenRoad");
    }
    
    logger_->info(utl::ECO, 1, "Initializing RL-based ECO flow");
    
    // Get the database from OpenROAD
    odb::dbDatabase* db = openroad_->getDb();
    if (!db) {
        logger_->warn(utl::ECO, 100, "No database loaded. Please read a design first.");
        return;
    }
    
    // Verify we have a valid design
    odb::dbChip* chip = db->getChip();
    if (!chip) {
        logger_->warn(utl::ECO, 101, "No chip in database. Please read a design first.");
        return;
    }
    
    odb::dbBlock* block = chip->getBlock();
    if (!block) {
        logger_->warn(utl::ECO, 102, "No block in database. Please read a design first.");
        return;
    }
    
    // Get STA from OpenROAD (optional - warn if not available)
    sta::dbSta* sta = openroad_->getSta();
    if (!sta) {
        logger_->warn(utl::ECO, 103, "STA not available - timing-driven ECO features will be disabled");
    }
    
    // Initialize the design manager
    try {
        design_manager_ = std::make_unique<EcoDesignManager>(db, sta, logger_);
        logger_->info(utl::ECO, 2, "Design manager initialized successfully");
    } catch (const std::exception& e) {
        logger_->error(utl::ECO, 104, "Failed to initialize design manager: {}", e.what());
        return;
    }
    
    // Initialize other components if needed
    // eco_agent_ = std::make_unique<EcoAgent>(logger_);
    // timing_analyzer_ = std::make_unique<EcoTimingAnalyzer>(sta, logger_);
    
    // Log some basic design statistics
    if (design_manager_ && block) {
        logger_->info(utl::ECO, 3, "Design: {}", block->getName());
        logger_->info(utl::ECO, 4, "Number of instances: {}", block->getInsts().size());
        logger_->info(utl::ECO, 5, "Number of nets: {}", block->getNets().size());
    }
    
    logger_->info(utl::ECO, 6, "RL-based ECO flow initialized successfully");
}

  

  
void RlEco::loadEcoChanges(const char* filename)
{
  if (!design_manager_) {
    logger_->error(utl::RLE, 28, "ECO RL not initialized");
  }
  
  design_manager_->parseChangeFile(filename);
  logger_->info(utl::RLE, 2, "Loaded {} ECO changes from {}", 
                design_manager_->getTotalChanges(), filename);
}

void RlEco::identifySpareCells()
{
  if (!design_manager_) {
    logger_->error(utl::RLE, 3, "ECO RL not initialized");
  }
  design_manager_->identifySpareCells();
  int num_spare = design_manager_->getIdentifiedSpareCells().size();
  logger_->info(utl::RLE, 4, "Identified {} spare cells", num_spare);
}

void RlEco::trainAgent(int episodes, 
                       double learning_rate,
                       double epsilon,
                       double gamma)
{
  if (!design_manager_ || design_manager_->getTotalChanges() == 0) {
    logger_->error(utl::RLE, 5, "No ECO changes loaded");
  }
  
  // Create Q-learning configuration
  QLearningConfig config;
  config.hidden_layers = hidden_layers_;
  config.learning_rate = learning_rate;
  config.epsilon_start = epsilon;
  config.epsilon_end = epsilon * 0.01;  // End at 1% of start value
  config.gamma = gamma;
  config.batch_size = batch_size_;
  config.replay_buffer_size = replay_buffer_size_;
  config.max_episodes = episodes;
  
  // Set reward weights
  config.timing_weight = timing_weight_;
  config.wirelength_weight = wirelength_weight_;
  config.congestion_weight = congestion_weight_;
  
  // Create trainer
  trainer_ = std::make_unique<EcoQLearningTrainer>(design_manager_, config);
  
  logger_->info(utl::RLE, 6, "Starting RL training:");
  logger_->info(utl::RLE, 7, "  Episodes: {}", episodes);
  logger_->info(utl::RLE, 8, "  Learning rate: {}", learning_rate);
  logger_->info(utl::RLE, 9, "  Initial epsilon: {}", epsilon);
  logger_->info(utl::RLE, 10, "  Hidden layers: {} {} ", 
                hidden_layers_[0], hidden_layers_[1]);
  
  // Enable checkpointing if configured
  if (checkpoint_enabled_) {
    // Create checkpoint directory if it doesn't exist
    std::filesystem::create_directories(checkpoint_dir_);
  }
  
  // Custom training loop with checkpointing
  for (int episode = 0; episode < episodes; ++episode) {
    trainer_->trainEpisode();
    
    // Progress reporting
    if (episode % 100 == 0) {
      trainer_->printTrainingProgress(episode);
    }
    
    // Checkpointing
    if (checkpoint_enabled_ && episode > 0 && episode % checkpoint_interval_ == 0) {
      std::string checkpoint_path = checkpoint_dir_ + "/checkpoint_episode_" + 
                                    std::to_string(episode) + ".ckpt";
      trainer_->saveAgent(checkpoint_path);
      logger_->info(utl::RLE, 11, "Saved checkpoint at episode {}", episode);
    }
    
    // Allow TCL events
    if (episode % 10 == 0) {
      Tcl_DoOneEvent(TCL_DONT_WAIT);
    }
  }
  
  // Print final training report
  logger_->info(utl::RLE, 12, "\n{}", trainer_->generateTrainingReport());
}

void RlEco::applyEcoWithRL()
{
  if (!trainer_) {
    logger_->error(utl::RLE, 14, "Agent not trained. Run train_agent first.");
  }
  
  // Get best move sequence from trainer
  auto best_moves = trainer_->getBestMoveSequence();
  
  logger_->info(utl::RLE, 15, "Applying {} ECO moves from RL agent", 
                best_moves.size());
  
  // Apply the moves
  for (const auto& move : best_moves) {
    design_manager_->applyMove(move);
  }
  
  // Report results
  reportMetrics();
}

void RlEco::saveModel(const char* filepath)
{
  if (!trainer_) {
    logger_->error(utl::RLE, 16, "No trained model to save");
  }
  trainer_->saveAgent(filepath);
  logger_->info(utl::RLE, 17, "Saved model to {}", filepath);
}

void RlEco::loadModel(const char* filepath)
{
  if (!design_manager_) {
    logger_->error(utl::RLE, 18, "ECO RL not initialized");
  }
  
  // Create trainer with default config if it doesn't exist
  if (!trainer_) {
    QLearningConfig config;
    config.hidden_layers = hidden_layers_;
    trainer_ = std::make_unique<EcoQLearningTrainer>(design_manager_, config);
  }
  
  trainer_->loadAgent(filepath);
  logger_->info(utl::RLE, 19, "Loaded model from {}", filepath);
}

void RlEco::reportMetrics() const
{
  logger_->info(utl::RLE, 20, "ECO Placement Results:");
  logger_->info(utl::RLE, 21, "  Completion rate: {:.1f}%", getCompletionRate());
  logger_->info(utl::RLE, 22, "  Timing improvement: {:.1f} ps", getTimingImprovement());
  logger_->info(utl::RLE, 23, "  Wirelength increase: {:.1f}%", getWirelengthIncrease());
  logger_->info(utl::RLE, 24, "  Congestion score: {:.2f}", getCongestionScore());
  
  if (trainer_) {
    logger_->info(utl::RLE, 25, "  Best episode reward: {:.2f}", 
                  trainer_->getBestReward());
  }
}

// Configuration setters
void RlEco::setReplayBufferSize(int size)
{
  replay_buffer_size_ = size;
}

void RlEco::setBatchSize(int size)
{
  batch_size_ = size;
}

void RlEco::setHiddenLayers(const std::vector<int>& layers)
{
  printf("In set hidden layers\n");
  hidden_layers_.clear();
  for (int size : layers) {
    hidden_layers_.push_back(static_cast<size_t>(size));
  }
}

void RlEco::setRewardWeights(double timing_weight,
                             double wirelength_weight,
                             double congestion_weight)
{
  timing_weight_ = timing_weight;
  wirelength_weight_ = wirelength_weight;
  congestion_weight_ = congestion_weight;
}

// Checkpointing
void RlEco::enableCheckpoints(const char* directory, int interval)
{
  checkpoint_enabled_ = true;
  checkpoint_dir_ = directory;
  checkpoint_interval_ = interval;
}

void RlEco::disableCheckpoints()
{
  checkpoint_enabled_ = false;
}

// Other methods...
void RlEco::clearEcoChanges()
{
  if (design_manager_) {
    design_manager_->clearChanges();
    trainer_.reset();  // Clear trainer when changes are cleared
  }
}

int RlEco::getNumChanges() const
{
  return design_manager_ ? design_manager_->getTotalChanges() : 0;
}

void RlEco::addSparePattern(const char* pattern)
{
  if (design_manager_) {
    design_manager_->addSparePattern(pattern);
  }
}

void RlEco::clearSparePatterns()
{
  if (design_manager_) {
    design_manager_->clearSparePatterns();
  }
}

int RlEco::getNumSpareCells() const
{
  return design_manager_ ? design_manager_->getIdentifiedSpareCells().size() : 0;
}

// Getters for metrics
float RlEco::getCompletionRate() const
{
  if (!design_manager_) return 0.0;
  return design_manager_->getCompletionRate() * 100.0;
}

float RlEco::getTimingImprovement() const
{
  if (!design_manager_) return 0.0;
  return design_manager_->getTimingImprovement();
}

float RlEco::getWirelengthIncrease() const
{
  if (!design_manager_) return 0.0;
  return design_manager_->getWirelengthIncrease() * 100.0;
}

float RlEco::getCongestionScore() const
{
  if (!design_manager_) return 0.0;
  return design_manager_->getCongestionScore();
}

void RlEco::applyEcoGreedy()
{
  if (!design_manager_) {
    logger_->error(utl::RLE, 26, "ECO RL not initialized");
  }
  
  // Apply greedy baseline for comparison
  design_manager_->applyGreedyPlacement();
  reportMetrics();
}

#include <filesystem>
#include <chrono>
#include <random>
#include <algorithm>

void RlEco::loadTrainingDataset(const char* directory) {
    training_scenarios_.clear();
    
    if (!std::filesystem::exists(directory)) {
        logger_->error(utl::RLE, 30, "Training directory {} does not exist", directory);
        return;
    }
    
    // Scan for .eco files
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            auto ext = entry.path().extension().string();
            if (ext == ".eco" || ext == ".txt" || ext == ".changes") {
                training_scenarios_.push_back(entry.path().string());
            }
        }
    }
    
    // Also look for subdirectories with categories
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_directory()) {
            std::string category = entry.path().filename().string();
            for (const auto& subentry : std::filesystem::directory_iterator(entry.path())) {
                if (subentry.is_regular_file()) {
                    auto ext = subentry.path().extension().string();
                    if (ext == ".eco" || ext == ".txt" || ext == ".changes") {
                        training_scenarios_.push_back(subentry.path().string());
                    }
                }
            }
        }
    }
    
    logger_->info(utl::RLE, 31, "Loaded {} training scenarios from {}", 
                  training_scenarios_.size(), directory);
}

void RlEco::trainOnDataset(int epochs) {
    if (training_scenarios_.empty()) {
        logger_->error(utl::RLE, 32, "No training scenarios loaded");
        return;
    }
    
    if (!trainer_) {
        // Create trainer with config
        QLearningConfig config;
        config.hidden_layers = hidden_layers_;
        config.learning_rate = 0.001;
        config.batch_size = batch_size_;
        config.replay_buffer_size = replay_buffer_size_;
        config.max_episodes = epochs * training_scenarios_.size();
        trainer_ = std::make_unique<EcoQLearningTrainer>(design_manager_, config);
    }
    
    logger_->info(utl::RLE, 33, "Starting dataset training: {} epochs on {} scenarios", 
                  epochs, training_scenarios_.size());
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    for (int epoch = 0; epoch < epochs; ++epoch) {
        // Shuffle scenarios each epoch
        std::shuffle(training_scenarios_.begin(), training_scenarios_.end(), gen);
        
        double epoch_total_reward = 0.0;
        int successful_scenarios = 0;
        
        for (size_t i = 0; i < training_scenarios_.size(); ++i) {
            const auto& scenario = training_scenarios_[i];
            
            // Load scenario
            design_manager_->reset();
            bool loaded = design_manager_->parseChangeFile(scenario.c_str());
            
            if (!loaded) {
                logger_->warn(utl::RLE, 34, "Failed to load scenario: {}", scenario);
                continue;
            }
            
            // Ensure spare cells are identified
            if (design_manager_->getIdentifiedSpareCells().empty()) {
                design_manager_->identifySpareCells();
            }
            
            // Train one episode on this scenario
            trainer_->trainEpisode();
            
            // Track progress
            auto& stats = trainer_->getStats();
            if (!stats.episode_rewards.empty()) {
                epoch_total_reward += stats.episode_rewards.back();
                successful_scenarios++;
            }
            
            // Progress update every 10 scenarios
            if ((i + 1) % 10 == 0) {
                logger_->info(utl::RLE, 35, "Epoch {}: Processed {}/{} scenarios", 
                             epoch + 1, i + 1, training_scenarios_.size());
            }
        }
        
        // Epoch summary
        double avg_reward = successful_scenarios > 0 ? 
                           epoch_total_reward / successful_scenarios : 0.0;
        logger_->info(utl::RLE, 36, "Epoch {} complete. Avg reward: {:.2f}", 
                     epoch + 1, avg_reward);
        
        // Decay epsilon
        trainer_->decayEpsilon();
        
        // Save checkpoint every 10 epochs
        if ((epoch + 1) % 10 == 0) {
            std::string checkpoint = "dataset_checkpoint_epoch_" + 
                                   std::to_string(epoch + 1) + ".model";
            trainer_->saveAgent(checkpoint);
            logger_->info(utl::RLE, 37, "Saved checkpoint: {}", checkpoint);
        }
    }
    
    has_trained_model_ = true;
    logger_->info(utl::RLE, 38, "Dataset training complete!");
}

void RlEco::benchmark(const char* method) {
    if (!design_manager_ || design_manager_->getTotalChanges() == 0) {
        logger_->error(utl::ECO, 50, "No ECO changes loaded for benchmarking");
        return;
    }
    
    logger_->info(utl::ECO, 51, "Running benchmark: RL vs {}", method);
    
    // Save initial state
    auto initial_metrics = getMetrics();
    
    // Run baseline method
    design_manager_->reset();
    auto baseline_start = std::chrono::high_resolution_clock::now();
    
    if (strcmp(method, "greedy") == 0) {
        design_manager_->applyGreedyPlacement();
    } else {
        logger_->error(utl::ECO, 52, "Unknown benchmark method: {}", method);
        return;
    }
    
    auto baseline_end = std::chrono::high_resolution_clock::now();
    auto baseline_metrics = getMetrics();
    auto baseline_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        baseline_end - baseline_start).count();
    
    // Run RL method
    design_manager_->reset();
    auto rl_start = std::chrono::high_resolution_clock::now();
    
    if (has_trained_model_) {
        // Use currently loaded ECO changes
        applyEcoWithRL();
    } else {
        logger_->warn(utl::ECO, 53, "No trained model, training quick model...");
        trainAgent(100, 0.001, 0.1, 0.99);
        applyEcoWithRL();
    }
    
    auto rl_end = std::chrono::high_resolution_clock::now();
    auto rl_metrics = getMetrics();
    auto rl_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        rl_end - rl_start).count();
    
    // Report comparison
    logger_->info(utl::ECO, 54, "\nBenchmark Results:");
    logger_->info(utl::ECO, 55, "================");
    logger_->info(utl::ECO, 56, "Method          | Completion | Timing Imp | WL Inc  | Time (ms)");
    logger_->info(utl::ECO, 57, "----------------|------------|------------|---------|----------");
    logger_->info(utl::ECO, 58, "{:15} | {:9.1f}% | {:10.1f} | {:6.1f}% | {:9}",
                  method, baseline_metrics["completion"], baseline_metrics["timing"],
                  baseline_metrics["wirelength"], baseline_time);
    logger_->info(utl::ECO, 59, "{:15} | {:9.1f}% | {:10.1f} | {:6.1f}% | {:9}",
                  "RL", rl_metrics["completion"], rl_metrics["timing"],
                  rl_metrics["wirelength"], rl_time);
    logger_->info(utl::ECO, 60, "\nImprovement: {:.1f}% better completion, {:.1f}x faster",
                  rl_metrics["completion"] - baseline_metrics["completion"],
                  static_cast<double>(baseline_time) / rl_time);
}

std::map<std::string, double> RlEco::getMetrics() const {
    std::map<std::string, double> metrics;
    
    if (design_manager_) {
        metrics["completion"] = getCompletionRate();
        metrics["timing"] = getTimingImprovement();
        metrics["wirelength"] = getWirelengthIncrease();
        metrics["congestion"] = getCongestionScore();
        metrics["spare_cells_used"] = design_manager_->getTotalChanges() - 
                                     design_manager_->getNumUnimplementedChanges();
        metrics["inference_time_ms"] = getInferenceTimeMs();
    }
    
    return metrics;
}


// In RlEco.h:
void inferOptimalPlacement(const char* variant_eco_file);

// In RlEco.cpp:
void RlEco::inferOptimalPlacement(const char* variant_eco_file) {
  /*
    if (!has_trained_model_ || !trainer_) {
        logger_->error(utl::RLE, 40, "No trained model loaded. Load or train a model first.");
        return;
    }
    
    // Load the variant's ECO requirements
    design_manager_->reset();
    bool loaded = design_manager_->parseChangeFile(variant_eco_file);
    if (!loaded) {
        logger_->error(utl::RLE, 41, "Failed to load ECO file: {}", variant_eco_file);
        return;
    }
    
    // Ensure spare cells are identified
    if (design_manager_->getIdentifiedSpareCells().empty()) {
        design_manager_->identifySpareCells();
        if (design_manager_->getIdentifiedSpareCells().empty()) {
            logger_->error(utl::RLE, 42, "No spare cells found in design");
            return;
        }
    }
    
    // Set agent to inference mode (no exploration, no learning)
    trainer_->getAgent()->setInferenceMode(true);
    
    // Create environment for this variant
    auto env = std::make_unique<EcoRLEnvironment>(design_manager_, logger_);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Initialize state
    auto state = env->reset();
    
    // Track moves and results
    std::vector<EcoMove> inference_moves;
    std::vector<std::string> placement_commands;
    std::vector<std::string> routing_commands;
    int changes_implemented = 0;
    int changes_skipped = 0;
    
    // Run inference loop
    while (!env->isDone() && inference_moves.size() < config_.max_steps_per_episode) {
        // Get current change being processed
        auto* current_change = design_manager_->getCurrentChange();
        if (!current_change) {
            break;
        }
        
        // Get state features
        auto state_features = state.toFeatureVector();
        
        // Get valid actions for current state
        auto valid_actions = env->getValidActions(state);
        if (valid_actions.empty()) {
            logger_->warn(utl::RLE, 43, "No valid actions for current state");
            break;
        }
        
        // Convert to indices for the agent
        std::vector<size_t> valid_indices;
        for (const auto& action : valid_actions) {
            valid_indices.push_back(action.toIndex());
        }
        
        // Get best action from trained model (no exploration in inference mode)
        size_t best_action_idx = trainer_->getAgent()->selectAction(
            state_features, valid_indices, false); // false = no training/exploration
        
        // Convert back to action
        EcoAction best_action = EcoAction::fromIndex(best_action_idx);
        
        // Execute the action
        auto [next_state, reward] = env->step(best_action);
        
        // Track what was done
        if (best_action.type == EcoAction::USE_SPARE_CELL) {
            auto spare_cells = design_manager_->getAvailableSpareCells();
            if (best_action.spare_cell_index < spare_cells.size()) {
                const auto& spare = spare_cells[best_action.spare_cell_index];
                
                // Create placement command
                auto gate_change = dynamic_cast<EcoGateChange*>(current_change);
                if (gate_change) {
                    auto inst_change = gate_change->getInstanceChange();
                    std::stringstream cmd;
                    cmd << "# Assign spare cell " << spare.name 
                        << " for new instance " << inst_change->getInfo().name << "\n";
                    cmd << "set_cell_location -inst " << spare.name 
                        << " -location {" << spare.x << " " << spare.y << "}\n";
                    cmd << "set_instance_placement_status -inst " << spare.name << " -status FIRM\n";
                    placement_commands.push_back(cmd.str());
                    
                    changes_implemented++;
                }
            }
        } else if (best_action.type == EcoAction::REROUTE) {
            auto net_change = dynamic_cast<EcoNetChange*>(current_change);
            if (net_change) {
                auto conn_change = net_change->getConnectionChange();
                std::stringstream cmd;
                cmd << "# Reroute for connection change\n";
                cmd << "route_eco -net " << conn_change->getInfo().net_name << "\n";
                routing_commands.push_back(cmd.str());
                
                changes_implemented++;
            }
        } else if (best_action.type == EcoAction::SKIP_CHANGE) {
            changes_skipped++;
            logger_->info(utl::RLE, 44, "Skipped change: {}", current_change->getDescription());
        }
        
        // Record the move
        EcoMove move;
        move.type = convertActionToMoveType(best_action);
        if (best_action.type == EcoAction::USE_SPARE_CELL) {
            auto spare_cells = design_manager_->getAvailableSpareCells();
            if (best_action.spare_cell_index < spare_cells.size()) {
                move.spare_cell_name = spare_cells[best_action.spare_cell_index].name;
                move.instance_name = current_change->getDescription();
            }
        }
        inference_moves.push_back(move);
        
        // Update state for next iteration
        state = next_state;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    inference_time_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();
    
    // Reset inference mode
    trainer_->getAgent()->setInferenceMode(false);
    
    // Generate output files
    std::string base_name = std::filesystem::path(variant_eco_file).stem().string();
    
    // Generate placement script
    std::string placement_file = base_name + "_placement.tcl";
    std::ofstream place_out(placement_file);
    place_out << "# ECO Placement Script for " << base_name << "\n";
    place_out << "# Generated by RL ECO tool at " << getCurrentTimestamp() << "\n";
    place_out << "# Total changes: " << design_manager_->getTotalChanges() << "\n";
    place_out << "# Implemented: " << changes_implemented << "\n";
    place_out << "# Skipped: " << changes_skipped << "\n\n";
    
    for (const auto& cmd : placement_commands) {
        place_out << cmd << "\n";
    }
    place_out.close();
    
    // Generate routing script
    std::string routing_file = base_name + "_routing.tcl";
    std::ofstream route_out(routing_file);
    route_out << "# ECO Routing Script for " << base_name << "\n";
    route_out << "# Metal-only changes\n";
    route_out << "# Generated by RL ECO tool at " << getCurrentTimestamp() << "\n\n";
    
    route_out << "# Set routing layers for ECO (metal-only)\n";
    route_out << "set_routing_layers -signal M2-M5\n\n";
    
    for (const auto& cmd : routing_commands) {
        route_out << cmd << "\n";
    }
    route_out.close();
    
    // Generate summary report
    std::string report_file = base_name + "_summary.rpt";
    std::ofstream report_out(report_file);
    report_out << "ECO Implementation Summary\n";
    report_out << "==========================\n\n";
    report_out << "Variant: " << base_name << "\n";
    report_out << "ECO file: " << variant_eco_file << "\n";
    report_out << "Inference time: " << inference_time_ms_ << " ms\n\n";
    
    report_out << "Changes Summary:\n";
    report_out << "  Total changes required: " << design_manager_->getTotalChanges() << "\n";
    report_out << "  Changes implemented: " << changes_implemented << "\n";
    report_out << "  Changes skipped: " << changes_skipped << "\n";
    report_out << "  Completion rate: " << std::fixed << std::setprecision(1) 
               << (100.0 * changes_implemented / design_manager_->getTotalChanges()) << "%\n\n";
    
    report_out << "Spare Cell Usage:\n";
    auto spare_usage = design_manager_->getSpareCellUsageReport();
    for (const auto& [type, count] : spare_usage) {
        report_out << "  " << type << ": " << count << " cells used\n";
    }
    
    report_out << "\nTiming Impact:\n";
    report_out << "  Worst slack: " << design_manager_->getWorstSlack() << " ns\n";
    report_out << "  Timing improvement: " << getTimingImprovement() << " ps\n";
    
    report_out << "\nPhysical Impact:\n";
    report_out << "  Wirelength increase: " << std::fixed << std::setprecision(2) 
               << getWirelengthIncrease() << "%\n";
    report_out << "  Congestion score: " << getCongestionScore() << "\n";
    
    report_out.close();
    
    // Log results
    logger_->info(utl::RLE, 45, "Inference complete for variant: {}", base_name);
    logger_->info(utl::RLE, 46, "  Completion rate: {:.1f}%", 
                  100.0 * changes_implemented / design_manager_->getTotalChanges());
    logger_->info(utl::RLE, 47, "  Spare cells used: {}", changes_implemented);
    logger_->info(utl::RLE, 48, "  Inference time: {} ms", inference_time_ms_);
    logger_->info(utl::RLE, 49, "  Generated files:");
    logger_->info(utl::RLE, 50, "    - {}", placement_file);
    logger_->info(utl::RLE, 51, "    - {}", routing_file);
    logger_->info(utl::RLE, 52, "    - {}", report_file);
    
    // Store the inference results for potential further analysis
    last_inference_moves_ = inference_moves;
    last_inference_completion_rate_ = 100.0 * changes_implemented / design_manager_->getTotalChanges();
  */
}

/*
// Helper method to convert EcoAction to EcoMove::MoveType
EcoMove::MoveType RlEco::convertActionToMoveType(const EcoAction& action) {
    switch (action.type) {
        case EcoAction::USE_SPARE_CELL:
            return EcoMove::USE_SPARE_CELL;
        case EcoAction::REROUTE:
            return EcoMove::CONNECT_NET;
        case EcoAction::BUFFER_INSERT:
            return EcoMove::BUFFER_NET;
        case EcoAction::SKIP_CHANGE:
            return EcoMove::SKIP;
        default:
            return EcoMove::SKIP;
    }
}
*/
// Helper to get current timestamp
std::string RlEco::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}


double RlEco::getInferenceTimeMs() const {
  return 0.0;
}

void RlEco::applySimpleEco() {
    if (!design_manager_) {
        logger_->error(utl::RLE, 401, "Design manager not initialized");
        return;
    }
    
    if (design_manager_->getTotalChanges() == 0) {
        logger_->error(utl::RLE, 402, "No ECO changes loaded");
        return;
    }
    
    design_manager_->applyAllChanges();
}

void RlEco::writeDEF(const char* filename) {
    if (!design_manager_) {
        logger_->error(utl::RLE, 403, "Design manager not initialized");
        return;
    }
    
    design_manager_->writeDEF(filename);
}

// Add these methods to RlEco class


void RlEco::eco_cell_changes() {
  bool success = design_manager_->applyEcoChanges();  // Apply instance changes
  (void)success;
}

void RlEco::eco_route_changes() {
    if (!design_manager_) {
        logger_->error(utl::RLE, 500, "Design manager not initialized");
        return;
    }
    
    bool success = design_manager_-> eco_route_changes();
    
    if (success) {
        logger_->info(utl::RLE, 501, "ECO routing completed successfully");
    } else {
        logger_->error(utl::RLE, 502, "ECO routing failed");
    }
}

void RlEco::writeRoutedDEF(const char* filename) {
    if (!design_manager_) {
        logger_->error(utl::RLE, 503, "Design manager not initialized");
        return;
    }
    
    design_manager_->writeRoutedDEF(filename);
}


}


