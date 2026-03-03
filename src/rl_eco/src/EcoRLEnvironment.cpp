#include "rl_eco/EcoRLEnvironment.h"
#include "rl_eco/EcoDesignManager.h"
#include "rl_eco/EcoChangeSet.h"
#include "utl/Logger.h"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace eco {


std::vector<double> EcoState::toFeatureVectorEnhanced() const {
    std::vector<double> features;
    
    // Basic features (normalized)
    features.push_back(static_cast<double>(num_unimplemented_changes) / 100.0);
    features.push_back(static_cast<double>(num_available_spare_cells) / 1000.0);
    features.push_back(total_spare_cells_available / 1000.0);
    features.push_back((current_timing_slack + 1.0) / 2.0);  // Normalize to [0,1]
    features.push_back(total_wirelength / 10000000.0);
    features.push_back(congestion_metric);
    
    // Change type encoding (one-hot, 10 dimensions)
    std::vector<double> change_type_encoding(10, 0.0);
    if (current_change_type == "gate_change") change_type_encoding[0] = 1.0;
    else if (current_change_type == "net_change") change_type_encoding[1] = 1.0;
    else if (current_change_type == "timing_fix") change_type_encoding[2] = 1.0;
    else if (current_change_type == "port_change") change_type_encoding[3] = 1.0;
    else change_type_encoding[9] = 1.0;  // Other/unknown
    features.insert(features.end(), change_type_encoding.begin(), change_type_encoding.end());
    
    // Required cell type encoding (one-hot, 20 dimensions for common types)
    std::vector<double> cell_type_encoding(20, 0.0);
    if (!required_cell_type.empty() && !cell_type_list.empty()) {
        auto it = std::find(cell_type_list.begin(), cell_type_list.end(), required_cell_type);
        if (it != cell_type_list.end()) {
            size_t idx = std::distance(cell_type_list.begin(), it);
            if (idx < 20) cell_type_encoding[idx] = 1.0;
        }
    }
    features.insert(features.end(), cell_type_encoding.begin(), cell_type_encoding.end());
    
    // Spare cells by type (normalized counts, 20 dimensions)
    for (int i = 0; i < 20; ++i) {
        if (i < spare_cells_by_type.size()) {
            features.push_back(spare_cells_by_type[i] / 100.0);
        } else {
            features.push_back(0.0);
        }
    }
    
    // Distance features (5 nearest compatible spare cells)
    for (int i = 0; i < 5; ++i) {
        if (i < spare_cell_distances.size()) {
            features.push_back(std::exp(-spare_cell_distances[i] / 1000.0));  // Exponential decay
        } else {
            features.push_back(0.0);
        }
    }
    
    // Compatibility score with nearest spare
    features.push_back(distance_to_nearest_compatible > 0 ? 
                      std::exp(-distance_to_nearest_compatible / 1000.0) : 0.0);
    
    // Change complexity
    features.push_back(static_cast<double>(change_complexity) / 10.0);
    
    return features;
}

  
// EcoState implementation
std::vector<double> EcoState::toFeatureVector() const {
    std::vector<double> features;
    
    // Basic features
    features.push_back(static_cast<double>(num_unimplemented_changes));
    features.push_back(static_cast<double>(num_available_spare_cells));
    features.push_back(current_timing_slack);
    features.push_back(total_wirelength / 1000000.0);  // Normalize
    features.push_back(congestion_metric);
    
    // Change type as one-hot encoding
    std::vector<double> change_type_encoding(5, 0.0);
    if (current_change_type == "gate_change") change_type_encoding[0] = 1.0;
    else if (current_change_type == "net_change") change_type_encoding[1] = 1.0;
    else if (current_change_type == "port_change") change_type_encoding[2] = 1.0;
    else if (current_change_type == "timing_fix") change_type_encoding[3] = 1.0;
    else change_type_encoding[4] = 1.0;
    features.insert(features.end(), change_type_encoding.begin(), change_type_encoding.end());
    
    features.push_back(static_cast<double>(change_complexity));
    
    // Add top 5 nearest spare cell distances (normalized)
    for (int i = 0; i < 5; ++i) {
        if (i < spare_cell_distances.size()) {
            features.push_back(spare_cell_distances[i] / 1000.0);  // Normalize
        } else {
            features.push_back(0.0);
        }
    }
    
    return features;
}

std::string EcoState::hash() const {
    std::stringstream ss;
    ss << num_unimplemented_changes << "_"
       << num_available_spare_cells << "_"
       << static_cast<int>(current_timing_slack * 100) << "_"
       << current_change_type;
    return ss.str();
}

// EcoAction implementation
std::string EcoAction::toString() const {
    switch (type) {
        case USE_SPARE_CELL: return "USE_SPARE_CELL_" + std::to_string(spare_cell_index);
        case REROUTE: return "REROUTE";
        case BUFFER_INSERT: return "BUFFER_INSERT_" + std::to_string(buffer_size);
        case RESIZE_GATE: return "RESIZE_GATE";
        case SKIP_CHANGE: return "SKIP_CHANGE";
        case NO_ACTION: return "NO_ACTION";
        default: return "UNKNOWN";
    }
}

// EcoRLEnvironment implementation
  EcoRLEnvironment::EcoRLEnvironment(std::shared_ptr<EcoDesignManager> manager,utl::Logger* logger)
    : design_manager_(manager),
      logger_(logger),
      done_(false),
      num_implemented_changes_(0) {

  initialize();
}
  

void  EcoRLEnvironment::initializeCellTypes(){
  

  auto cell_types = design_manager_->getAvailableSpareCellTypes();
  // Convert set to vector for consistent ordering
  discovered_cell_types_.assign(cell_types.begin(), cell_types.end());
  std::sort(discovered_cell_types_.begin(), discovered_cell_types_.end());
  std::map<std::string,int> cell_type_to_index;
  
  // Create mapping for fast lookup
  for (size_t i = 0; i < discovered_cell_types_.size(); ++i) {
    cell_type_to_index[discovered_cell_types_[i]] = i;
  }
    
  // Get type distribution for logging
  auto type_counts = design_manager_->getSpareCellTypeCount();
    
  // Log discovered types with counts
  logger_->info(utl::ECO, 200, "Discovered {} spare cell types:", discovered_cell_types_.size());
  for (const auto& [type, count] : type_counts) {
    logger_->info(utl::ECO, 201, "  {} : {} cells", type, count);
  }
}

  void EcoRLEnvironment::initialize(){
     if (design_manager_->getIdentifiedSpareCells().empty()) {
            std::cerr << "Warning: No spare cells identified. Calling identifySpareCells()..." << std::endl;
            design_manager_->identifySpareCells();
        }
        
        // Initialize cell types from the identified spare cells
        initializeCellTypes();
        
        // Validate we have required data
        if (discovered_cell_types_.empty()) {
            throw std::runtime_error("No spare cell types discovered. Cannot proceed with RL environment.");
        }
        
        // Initialize other components
        total_changes_ = design_manager_->getTotalChanges();
        current_state_ = computeCurrentState();
  }

  

EcoState EcoRLEnvironment::reset() {
    design_manager_->reset();  // Reset to initial state
    done_ = false;
    num_implemented_changes_ = 0;
    //reinitialize cell types
    initializeCellTypes();
    current_state_ = computeCurrentState();
    return current_state_;
}

std::pair<EcoState, double> EcoRLEnvironment::step(const EcoAction& action) {

  logger_->info(utl::ECO, 999, "Action taken: {} (index {})", 
                  action.toString(), action.toIndex());
    
    EcoState prev_state = current_state_;
    
    // Execute action
    bool success = executeAction(action);
    (void)success;
    
    // Update state
    current_state_ = computeCurrentState();
    
    // Calculate reward
    double reward = calculateReward(prev_state, current_state_, action);
    
    // Check if done
    done_ = design_manager_->allChangesProcessed() || 
            !design_manager_->hasUnimplementedChanges();
    
    return {current_state_, reward};
}

bool EcoRLEnvironment::isDone() const {
    return done_;
}

std::vector<EcoAction> EcoRLEnvironment::getValidActions(const EcoState& state) const {
    std::vector<EcoAction> valid_actions;
    
    // Always can skip
    valid_actions.push_back(EcoAction(EcoAction::SKIP_CHANGE));
    
    if (state.num_unimplemented_changes == 0) {
        valid_actions.push_back(EcoAction(EcoAction::NO_ACTION));
        return valid_actions;
    }
    
    // Check available spare cells
    auto spare_cells = design_manager_->getAvailableSpareCells();
    for (size_t i = 0; i < spare_cells.size(); i++){
        valid_actions.push_back(EcoAction(EcoAction::USE_SPARE_CELL, i));
    }
    
    // Other actions based on change type
    if (state.current_change_type == "net_change") {
        valid_actions.push_back(EcoAction(EcoAction::REROUTE));
    }
    
    if (state.current_timing_slack < 0) {
        valid_actions.push_back(EcoAction(EcoAction::BUFFER_INSERT));
        valid_actions.push_back(EcoAction(EcoAction::RESIZE_GATE));
    }
    
    return valid_actions;
}

int EcoRLEnvironment::getStateSize() const {
  EcoState dummy_state;
  return dummy_state.toFeatureVectorEnhanced().size();
}

int EcoRLEnvironment::getActionSize() const {
  // Fixed actions + number of spare cells
  size_t num_spare_cells = design_manager_->getAvailableSpareCells().size();
  num_spare_cells = std::min(num_spare_cells, MAX_SPARE_CELLS_FOR_RL);
  return EcoAction::getFixedActionCount() + num_spare_cells;
}

double EcoRLEnvironment::calculateReward(const EcoState& prev_state,
                                        const EcoState& curr_state,
                                        const EcoAction& action) const {
    double reward = 0.0;

    //debug information
    // Calculate components
    bool change_implemented = curr_state.num_unimplemented_changes < prev_state.num_unimplemented_changes;
    double timing_improvement = curr_state.current_timing_slack - prev_state.current_timing_slack;
    double wirelength_increase = curr_state.total_wirelength - prev_state.total_wirelength;
    
    // Apply rewards/penalties
    if (change_implemented) {
        reward += CHANGE_IMPLEMENTED_REWARD;
        logger_->info(utl::ECO, 995, "Change implemented! +{}", CHANGE_IMPLEMENTED_REWARD);
    }
    
    if (timing_improvement > 0) {
        double timing_reward = TIMING_IMPROVEMENT_REWARD * timing_improvement;
        reward += timing_reward;
        logger_->info(utl::ECO, 994, "Timing improved by {:.2f}! +{:.2f}", 
                      timing_improvement, timing_reward);
    }
    
    if (action.type == EcoAction::SKIP_CHANGE) {
        reward += SKIP_PENALTY;
        logger_->info(utl::ECO, 993, "Skipped change! {}", SKIP_PENALTY);
    }
    

    
    // Reward for implementing a change
    if (curr_state.num_unimplemented_changes < prev_state.num_unimplemented_changes) {
        reward += CHANGE_IMPLEMENTED_REWARD;
        
        // Bonus for timing improvement
        if (curr_state.current_timing_slack > prev_state.current_timing_slack) {
            reward += TIMING_IMPROVEMENT_REWARD * 
                     (curr_state.current_timing_slack - prev_state.current_timing_slack);
        }
    }
    
    // Penalties
    if (action.type == EcoAction::SKIP_CHANGE) {
        reward += SKIP_PENALTY;
    }
    
    // Wirelength penalty
    wirelength_increase = curr_state.total_wirelength - prev_state.total_wirelength;
    if (wirelength_increase > 0) {
        reward += WIRELENGTH_PENALTY * wirelength_increase;
    }
    
    // Congestion penalty
    double congestion_increase = curr_state.congestion_metric - prev_state.congestion_metric;
    if (congestion_increase > 0) {
        reward += CONGESTION_PENALTY * congestion_increase;
    }
    
    logger_->info(utl::ECO, 999, "Reward: action={}, reward={}, impl={}, skip={}",
		  action.toString(), reward, 
		  curr_state.num_unimplemented_changes < prev_state.num_unimplemented_changes,
		  action.type == EcoAction::SKIP_CHANGE);
    
    return reward;
}

EcoState EcoRLEnvironment::computeCurrentState() const {
    EcoState state;
    
    // Existing state computation...
    state.num_unimplemented_changes = design_manager_->getNumUnimplementedChanges();
    state.num_available_spare_cells = design_manager_->getNumAvailableSpareCells();
    state.current_timing_slack = design_manager_->getWorstSlack();
    state.total_wirelength = design_manager_->getTotalWirelength();
    state.congestion_metric = design_manager_->getCongestionMetric();
    
    // NEW: Get spare cell type information
    state.cell_type_list = discovered_cell_types_;
    state.spare_cells_by_type.clear();
    auto type_counts = design_manager_->getSpareCellTypeCount();
    for (const auto& type : discovered_cell_types_) {
        auto it = type_counts.find(type);
        state.spare_cells_by_type.push_back(it != type_counts.end() ? it->second : 0);
    }
    state.total_spare_cells_available = design_manager_->getNumAvailableSpareCells();
    
    auto* current_change = design_manager_->getCurrentChange();
    if (current_change) {
        state.current_change_type = current_change->getType();
        state.change_complexity = current_change->getComplexity();
        
        // NEW: Extract required cell type for gate changes
        auto gate_change = dynamic_cast<EcoGateChange*>(current_change);
        if (gate_change) {
            auto inst_change = gate_change->getInstanceChange();
            if (inst_change) {
                state.required_cell_type = design_manager_->extractBaseType(
                    inst_change->getInfo().cell_type);
            }
        }
        
        // Get distances to spare cells, prioritizing compatible ones
        state.spare_cell_distances.clear();
        state.distance_to_nearest_compatible = std::numeric_limits<double>::max();
        
        auto spare_cells = design_manager_->getAvailableSpareCells();
        std::vector<std::pair<double, std::string>> distance_type_pairs;
        
        for (const auto& cell : spare_cells) {
            double dist = design_manager_->getDistanceToChange(cell, current_change);
            std::string base_type = design_manager_->extractBaseType(cell.type);
            distance_type_pairs.push_back({dist, base_type});
            
            // Track nearest compatible
            if (base_type == state.required_cell_type) {
                state.distance_to_nearest_compatible = std::min(
                    state.distance_to_nearest_compatible, dist);
            }
        }
        
        // Sort by distance
        std::sort(distance_type_pairs.begin(), distance_type_pairs.end());
        
        // Store top distances
        for (const auto& [dist, type] : distance_type_pairs) {
            state.spare_cell_distances.push_back(dist);
            state.spare_cell_types.push_back(type);
            if (state.spare_cell_distances.size() >= 10) break;
        }
    } else {
        state.current_change_type = "none";
        state.change_complexity = 0;
        state.distance_to_nearest_compatible = 0.0;
    }
    
    return state;
}
  

bool EcoRLEnvironment::executeAction(const EcoAction& action) {
    auto* current_change = design_manager_->getCurrentChange();
    if (!current_change && action.type != EcoAction::NO_ACTION) {
        return false;
    }
    
    bool success = false;
    
    switch (action.type) {
        case EcoAction::USE_SPARE_CELL: {
            auto spare_cells = design_manager_->getAvailableSpareCells();
            if (action.spare_cell_index < spare_cells.size()) {
                success = design_manager_->implementChangeWithSpareCell(
                    current_change, spare_cells[action.spare_cell_index]);
                if (success) num_implemented_changes_++;
            }
            break;
        }
        
        case EcoAction::REROUTE:
            success = design_manager_->implementChangeWithReroute(current_change);
            if (success) num_implemented_changes_++;
            break;
            
        case EcoAction::BUFFER_INSERT:
            success = design_manager_->insertBuffer(current_change, action.buffer_size);
            if (success) num_implemented_changes_++;
            break;
            
        case EcoAction::RESIZE_GATE:
            success = design_manager_->resizeGate(current_change);
            if (success) num_implemented_changes_++;
            break;
            
        case EcoAction::SKIP_CHANGE:
            design_manager_->skipChange(current_change);
            success = true;
            break;
            
        case EcoAction::NO_ACTION:
            success = true;
            break;
    default:
            success = true;
            break;
    }
    
    if (success) {
        updateMetrics();
    }
    
    return success;
}

void EcoRLEnvironment::updateMetrics() {
    design_manager_->updateTimingMetrics();
    design_manager_->updatePhysicalMetrics();
}

} // namespace eco
