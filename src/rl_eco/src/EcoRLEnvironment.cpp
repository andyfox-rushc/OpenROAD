#include "ord/OpenRoad.hh"
#include "rl_eco/EcoTypes.h" 
#include "rl_eco/EcoRLEnvironment.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <chrono>
#include <set>
#include <map>

namespace eco {

  EcoRLEnvironment::~EcoRLEnvironment(){}
  
  EcoRLEnvironment::EcoRLEnvironment(std::shared_ptr<EcoDesignManager> manager, 
                                   rsz::Resizer* resizer,
                                   int num_critical_paths,
                                   int max_moves_per_episode)
    : design_manager_(manager)
    , resizer_(resizer)
    , num_critical_paths_(num_critical_paths)
    , max_moves_per_episode_(max_moves_per_episode)
    , current_move_(0)
    , episode_active_(false) {
}

void EcoRLEnvironment::reset() {
    // Reset episode tracking
    current_move_ = 0;
    episode_active_ = true;
    recent_improvements_.clear();
    
    // Initialize episode stats
    current_stats_ = EpisodeStats();
    current_stats_.initial_tns = design_manager_->evaluateTotalNegativeSlack();
    current_stats_.tns_trajectory.push_back(current_stats_.initial_tns);

    //reset the action / index look ups
    action2index_.clear();
    index2action_.clear();
}

DesignState EcoRLEnvironment::getCurrentState() {
    DesignState state;
    
    // Get global metrics
    state.tns = design_manager_->evaluateTotalNegativeSlack();
    state.wns = design_manager_->evaluateWorstNegativeSlack();
    state.total_area = design_manager_->evaluateDesignArea();
    state.total_power = design_manager_->evaluateTotalPower();
    
    // Analyze critical paths
    auto analysis = analyzeCriticalPaths();
    
    // Extract path-specific features
    state.critical_path_slacks.reserve(num_critical_paths_);
    state.path_cell_counts.reserve(num_critical_paths_);
    
    for (const auto& path : analysis.critical_paths) {
        state.critical_path_slacks.push_back(path.slack);
        state.path_cell_counts.push_back(path.instances.size());
    }
    
    // Pad if fewer paths than expected
    while (state.critical_path_slacks.size() < num_critical_paths_) {
        state.critical_path_slacks.push_back(0.0);
        state.path_cell_counts.push_back(0.0);
    }
    
    // Extract spare utilization features
    state.spare_cell_distances = extractSpareUtilization();
    
    // Move history
    state.recent_improvements.assign(recent_improvements_.begin(), 
                                   recent_improvements_.end());
    // Pad to fixed size (e.g., last 5 moves)
    while (state.recent_improvements.size() < 5) {
        state.recent_improvements.push_back(0.0);
    }
    
    state.moves_attempted = current_move_;
    state.moves_accepted = current_stats_.moves_accepted;
    
    return state;
}

  double EcoRLEnvironment::step(const std::shared_ptr<EcoAction> action) {

    if (!episode_active_ || isEpisodeDone()) {
        return 0.0;
    }
    
    // Record current metrics
    double prev_tns = design_manager_->evaluateTotalNegativeSlack();
    double prev_wns = design_manager_->evaluateWorstNegativeSlack();
    
    // Execute move with journaling
    resizer_->journalBegin();
    EcoDesignManager::MoveResult result = executeMove(action);
    
    // Calculate reward
    double reward = calculateReward(result, prev_tns, prev_wns);
    
    // Decide whether to accept the move
    bool should_accept = false;
    if (result.success && result.timing_improvement > min_improvement_threshold_) {
        should_accept = true;
    } else if (result.success && reward > 0) {
        // Accept small improvements if overall reward is positive
        should_accept = true;
    }
    
    if (should_accept) {
        acceptMove();
        current_stats_.moves_accepted++;
        recent_improvements_.push_back(result.timing_improvement);
    } else {
        rejectMove();
        current_stats_.moves_rejected++;
        recent_improvements_.push_back(0.0);
        reward = -0.1;  // Small penalty for rejected move
    }
    
    // Maintain sliding window
    if (recent_improvements_.size() > 5) {
        recent_improvements_.pop_front();
    }
    
    // Update episode tracking
    current_move_++;
    current_stats_.total_reward += reward;
    current_stats_.tns_trajectory.push_back(
        design_manager_->evaluateTotalNegativeSlack()
    );
    
    return reward;
}

PathAnalysis EcoRLEnvironment::analyzeCriticalPaths() {
    PathAnalysis analysis;
    
    // Get critical paths
    analysis.critical_paths = design_manager_->getCriticalPaths(num_critical_paths_);
    
    // Build instance-to-paths mapping and criticality scores
    for (int path_idx = 0; path_idx < analysis.critical_paths.size(); ++path_idx) {
        const auto& path = analysis.critical_paths[path_idx];
        double path_weight = std::exp(-path_idx * 0.1);  // Exponential decay
        
        for (const auto& inst : path.instances) {
	  std::string inst_name = inst -> getName();
            analysis.instance_to_paths[inst_name].push_back(path_idx);
            analysis.instance_criticality[inst_name] += path_weight * std::abs(path.slack);
        }
    }
    
    return analysis;
}


  std::vector<size_t> EcoRLEnvironment::getValidActionIndices(const DesignState& state){
    // Generate actual actions
    getValidActions(state);
    std::vector<size_t> valid_indices;
    for (auto i2a: index2action_){
      valid_indices.push_back(i2a.first);
    }
    return valid_indices;
  }

  std::shared_ptr<EcoAction> EcoRLEnvironment::getActionFromIndex(size_t index){
    std::map<size_t,std::shared_ptr<EcoAction> >::iterator it;
    it = index2action_.find(index); 
    if (it != index2action_.end()){
      return (*it).second;
    }
    return nullptr;
  }

  size_t EcoRLEnvironment::getIndexFromAction(const std::shared_ptr<EcoAction> action){
    std::map<std::shared_ptr<EcoAction>,size_t >::iterator it;
    it = action2index_.find(action);
    if (it != action2index_.end()){
      return (*it).second;
    }
    return MAX_ACTIONS;
  }


std::vector<std::shared_ptr<EcoAction> > EcoRLEnvironment::getValidActions(const DesignState& state) {

  auto start = std::chrono::steady_clock::now();
  auto analysis = analyzeCriticalPaths();
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double,std::milli> duration = end-start;
  //  printf("Time to analyze critical paths %.10f ms\n",duration);

  auto generateShannonActions(analysis);
  
  std::vector<std::shared_ptr<EcoAction>> all_actions;
  // Generate actions of each type
  start = std::chrono::steady_clock::now();    
  auto resize_actions = generateResizeActions(analysis);
  end = std::chrono::steady_clock::now();
  duration = end-start;
  //  printf("Time to generate Resize Actions %.10f ms \n",duration);

    auto rebuffer_actions = generateRebufferActions(analysis);
    auto load_split_actions = generateLoadSplitActions(analysis);
    auto retime_actions = generateRetimeActions(analysis);
    auto pipeline_actions = generatePipelineActions(analysis);
    auto logic_remap_actions = generateLogicRemapActions(analysis);
    
    // Combine all actions
    all_actions.insert(all_actions.end(), resize_actions.begin(), resize_actions.end());
    all_actions.insert(all_actions.end(), rebuffer_actions.begin(), rebuffer_actions.end());
    all_actions.insert(all_actions.end(), load_split_actions.begin(), load_split_actions.end());
    all_actions.insert(all_actions.end(), retime_actions.begin(), retime_actions.end());
    all_actions.insert(all_actions.end(), pipeline_actions.begin(), pipeline_actions.end());
    all_actions.insert(all_actions.end(), logic_remap_actions.begin(), logic_remap_actions.end());
    
    // Sort by predicted improvement
    std::sort(all_actions.begin(), all_actions.end(),
              [](const std::shared_ptr<EcoAction>& a, const std::shared_ptr<EcoAction>& b) {
                  return a->predicted_improvement > b->predicted_improvement;
              });
    
    // Limit total action space
    if (all_actions.size() > 200) {
        all_actions.resize(200);
    }

    //build the map of action to index
    size_t index=0;
    for (auto action: all_actions){
      action2index_[action]=index;
      index2action_[index]=action;
      index++;
    }

    max_action_size_ = all_actions.size() > max_action_size_ ? all_actions.size():max_action_size_;
    
    return all_actions;
}


  bool EcoRLEnvironment::IsShannonFeasible(const eco::PathInfo& path_info,
					   odb::dbITerm* &start_iterm,
					   odb::dbITerm* &end_iterm,
					   unsigned N){

    SpareCellsDictionary spare_cells_dictionary;
    design_manager_ -> populateSpareCellsDictionary(spare_cells_dictionary);
    
    sta::dbSta* sta = ord::OpenRoad::openRoad()->getSta();

    for (int inst_idx = 0; inst_idx < path_info.instances.size(); ++inst_idx) {
      odb::dbInst* inst = path_info.instances[inst_idx];
      if (design_manager_ -> isFeasibleMaster(inst -> getMaster() -> getName(),
					      spare_cells_dictionary)){
	design_manager_ -> markUsed(inst -> getMaster() -> getName(),
				    spare_cells_dictionary);
	printf("%s -> ", inst -> getName().c_str());
      }
    }
    return false;
  }
  
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateShannonActions(
						 const PathAnalysis& analysis) {
  for (int path_idx = 0; path_idx < analysis.critical_paths.size(); ++path_idx) {
        const auto& path = analysis.critical_paths[path_idx];
	odb::dbITerm* start_iterm=nullptr;
	odb::dbITerm* end_iterm=nullptr;
	
	if (IsShannonFeasible(path,start_iterm, end_iterm,EcoRLEnvironment::SHANNON_DEPTH)){
	  printf("Found shannon feasible split path\n");
	}
	sta::dbSta* sta = ord::OpenRoad::openRoad()->getSta();
		printf("Examining path %d:  slack %s ns\n",path.index,delayAsString(path.slack,sta));
		for (int inst_idx = 0; inst_idx < path.instances.size(); ++inst_idx) {
		  odb::dbInst* inst = path.instances[inst_idx];
		  printf("%s -> ", inst -> getName().c_str());
		}
		printf("\n");
  }  
}

  
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateResizeActions(
    const PathAnalysis& analysis) {

  //use the caching
  design_manager_ -> wireLengthCache(true);
  
  std::vector<std::shared_ptr<EcoAction>> actions;

    std::map<std::string,std::set<std::string> > compatible_masters_set;
    std::map<std::string,std::set<std::string> > ::iterator  compatible_masters_set_it;
    std::set<std::string> compatible_masters;
    
    // For each critical path
    for (int path_idx = 0; path_idx < analysis.critical_paths.size(); ++path_idx) {
        const auto& path = analysis.critical_paths[path_idx];

	sta::dbSta* sta = ord::OpenRoad::openRoad()->getSta();
		printf("Examining path %d:  slack %s ns\n",path.index,delayAsString(path.slack,sta));
		for (int inst_idx = 0; inst_idx < path.instances.size(); ++inst_idx) {
		  odb::dbInst* inst = path.instances[inst_idx];
		  printf("%s -> ", inst -> getName().c_str());
		}
		printf("\n");
	
        // For each instance in the path
        for (int inst_idx = 0; inst_idx < path.instances.size(); ++inst_idx) {
            odb::dbInst* inst = path.instances[inst_idx];
            std::string instance_name = inst->getName();
            odb::dbMaster* current_master = inst->getMaster();
            std::string current_master_name = current_master->getName();
	    double current_instance_area = design_manager_ -> getMasterArea(current_master_name);
            // Find nearby spare cells first
	    
	    long long  max_radius = design_manager_ -> getMaxRadius(inst);
	    
	    std::vector<std::shared_ptr<SpareCell>> nearby_spares = findNearbySpares(inst, max_radius);

	    printf("number of spares to explore: %d\n", nearby_spares.size());
	    
	    auto start = std::chrono::steady_clock::now();
	    //change to set
	    compatible_masters_set_it = compatible_masters_set.find(current_master_name);
	    if (compatible_masters_set_it != compatible_masters_set.end()){
	      compatible_masters = (*compatible_masters_set_it).second;
	    }
	    else{
	      compatible_masters_set[current_master_name] =
		design_manager_->getCompatibleMasters(current_master_name);
	      compatible_masters =  compatible_masters_set[current_master_name];
	    }
	    
	    auto end = std::chrono::steady_clock::now();
	    std::chrono::duration<double,std::milli> duration = end-start;
	    //	    printf("Time to get compatible masters %.10f ms\n",duration);


	    for (auto spare_candidate: nearby_spares){


	      
	      odb::dbInst* spare_inst = spare_candidate -> instance;
	      odb::dbMaster* spare_master = spare_candidate -> master;
	      std::string spare_master_name = spare_master -> getName();

	      //Is the candidate spare compatible with the master ?
	      bool compatible = false;
	      if (compatible_masters.find(spare_master_name) !=
		  compatible_masters.end()){
		compatible = true;
	      }
	      else{
		continue;
	      }
	      
	      bool is_downsize =design_manager_ -> getMasterArea(spare_master_name) <
		current_instance_area;
	      
	      if (is_downsize){
		continue;
	      }


	      start = std::chrono::steady_clock::now();
	      //Try the swap. Swap instance for spare
	      EcoDesignManager::MoveResult move_gain = 
		design_manager_->previewResize(inst, spare_inst);

		auto end = std::chrono::steady_clock::now();
		duration = end-start;


	      // Only create action if there's predicted improvement
                if (move_gain.timing_improvement > 0) {
                    // Create EcoAction with ResizeAction
                    auto action = std::make_shared<EcoAction>();
                    action->type = EcoAction::ActionType::RESIZE;
                    action->resize = std::make_unique<ResizeAction>();
                    action->resize->instance_name = instance_name;
                    action->resize->new_master = spare_master_name;
                    action->resize->critical_path_index = path_idx;
                    action->resize->instance_index_in_path = inst_idx;
                    
                    action->resize->spare_instance = spare_candidate -> instance -> getName(); 
                    
                    // Set metrics
                    action->predicted_improvement = move_gain.timing_improvement;
                    action->resize->predicted_improvement = action->predicted_improvement;
                    action->affected_critical_paths = 1;
                    action->confidence_score = 0.8;
                    
                    actions.push_back(action);
                }
	    }
        }
    }
    return actions;
}

  

  
  /*
    Estimate the value of an action. Assume values of actions computed during action generation.
  */
double EcoRLEnvironment::estimateActionValue(const std::shared_ptr<EcoAction>& action, 
                                            const PathAnalysis& analysis) {
    double value = 0.0;
    
    switch (action->type) {
    case EcoAction::ActionType::RESIZE: {
            if (action->resize) {
	      value = action -> predicted_improvement;
            break;
        }
    }        
    case EcoAction::ActionType::REBUFFER: {
            if (action->rebuffer) {
                // TODO: Estimate rebuffer value based on net length, criticality, etc.
            }
            break;
        }
        
        // TODO: Add estimation for other action types
        
        default:
            break;
    }
    
    return value;
}

  std::vector<std::shared_ptr<SpareCell> > EcoRLEnvironment::findNearbySpares(odb::dbInst* inst, long long radius) {
    return design_manager_->findSpareCellsNear(inst, radius);
}

  
//  
// Placeholder implementations for other action generators
//
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateRebufferActions(
    const PathAnalysis& analysis) {
    std::vector<std::shared_ptr<EcoAction>> actions;
    // TODO: Implement rebuffer action generation
    // Example structure:
    // auto action = std::make_shared<EcoAction>();
    // action->type = EcoAction::REBUFFER;
    // action->rebuffer = std::make_unique<RebufferAction>();
    // action->rebuffer->operation = RebufferAction::BufferOp::INSERT_BUFFER;
    // ...
    return actions;
}
 
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateLoadSplitActions(
    const PathAnalysis& analysis) {
    std::vector<std::shared_ptr<EcoAction>> actions;
    // TODO: Implement load split action generation
    return actions;
}
 
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateRetimeActions(
    const PathAnalysis& analysis) {
    std::vector<std::shared_ptr<EcoAction>> actions;
    // TODO: Implement retime action generation
    return actions;
}
 
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generatePipelineActions(
    const PathAnalysis& analysis) {
    std::vector<std::shared_ptr<EcoAction>> actions;
    // TODO: Implement pipeline action generation
    return actions;
}
 
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateLogicRemapActions(
    const PathAnalysis& analysis) {
    std::vector<std::shared_ptr<EcoAction>> actions;
    // TODO: Implement logic remap action generation
    return actions;
}


  
  double EcoRLEnvironment::calculateReward(const EcoDesignManager::MoveResult& result,
                                        double prev_tns,
                                        double prev_wns) {
    if (!result.success) {
        return -1.0;  // Penalty for invalid moves
    }
    
    // Timing improvement component (primary objective)
    double timing_reward = 0.0;
    if (result.timing_improvement > 0) {
        // Reward proportional to improvement, with diminishing returns
        timing_reward = std::log1p(result.timing_improvement / 10.0);  // 10ps scale
    } else {
        // Heavier penalty for timing degradation  
        timing_reward = result.timing_improvement / 5.0;
    }
    
    // TNS improvement component
    double current_tns = design_manager_->evaluateTotalNegativeSlack();
    double tns_improvement = prev_tns - current_tns;
    double tns_reward = tns_improvement / 100.0;  // Normalize by 100ps
    
    // Area penalty (minimize area increase)
    double area_penalty = 0.0;
    if (result.area_delta > 0) {
        area_penalty = -0.1 * (result.area_delta / 1000.0);  // per 1000 sq units
    }

    //TODO
    double power_penalty = 0.0;
    /*
    // Power penalty

    if (result.power_delta > 0) {
        power_penalty = -0.05 * (result.power_delta / 0.001);  // per mW
    }
    */
    // Combine components with weights
    double total_reward = 
        0.6 * timing_reward +
        0.3 * tns_reward +
        0.05 * area_penalty +
        0.05 * power_penalty;
    
    // Bonus for fixing critical paths
    if (current_tns < prev_tns * 0.9) {  // 10% improvement
        total_reward += 0.5;
    }
    
    return total_reward;
}


  EcoDesignManager::MoveResult EcoRLEnvironment::executeMove(const std::shared_ptr<EcoAction> action) {
    EcoDesignManager::MoveResult default_error;
    switch (action -> type){
    case EcoAction::ActionType::RESIZE:
      {
	std::string original_instance_name = action -> resize -> instance_name;
	std::string spare_instance_name = action -> resize -> spare_instance;
	odb::dbInst* original_instance = design_manager_ -> findInst(original_instance_name);
	odb::dbInst* new_instance = design_manager_ -> findInst(spare_instance_name);
	return design_manager_->performInstanceSwap(original_instance,
						new_instance);
      }
      break;
    default:
      break;
    }
    return default_error;
  }
  

void EcoRLEnvironment::acceptMove() {
    resizer_->journalEnd();
}

void EcoRLEnvironment::rejectMove() {
    resizer_->journalRestore();
    resizer_->journalEnd();
}

bool EcoRLEnvironment::isEpisodeDone() const {
    // Episode ends when:
    // 1. Maximum moves reached
    if (current_move_ >= max_moves_per_episode_) return true;
    
    // 2. TNS is close to zero (problem solved)
    if (std::abs(design_manager_->evaluateTotalNegativeSlack()) < 1.0) return true;
    
    // 3. No improvement in last N moves
    if (recent_improvements_.size() >= 5) {
        double sum = std::accumulate(recent_improvements_.begin(), 
                                   recent_improvements_.end(), 0.0);
        if (sum < 0.1) return true;  // Less than 0.1ps improvement in 5 moves
    }
    
    return false;
}

std::vector<double> EcoRLEnvironment::extractSpareUtilization() {
    // This would analyze spare cell availability and distribution
    // Simplified version:
    std::vector<double> features;
    
    auto spare_info = design_manager_->getSpareGateSummary();
    features.push_back(spare_info.total_count);
    features.push_back(spare_info.buffers);
    features.push_back(spare_info.inverters);
    features.push_back(spare_info.logic_gates);
    
    return features;
}

  size_t EcoRLEnvironment::getActionSize(){
    return index2action_.size();
  }

  size_t EcoRLEnvironment::getStateSize(){
    size_t ret = 4; //tns + wns + total_area + total_power
    ret += num_critical_paths_*2; //slacks + cell_counts
    auto spare_info = design_manager_->getSpareGateSummary();
    ret += spare_info.total_count; //the number of spare cells
    return ret;
  }

  
  std::vector<double> DesignState::toVector() const {
    std::vector<double> vec;
    printf("+++++++++++\n");
    printf("Features:\n");
    printf("Global Metrics tns %.10f wns %.10f area %.10f power %.10f \n",
	   tns,
	   wns,
	   total_area,
	   total_power);

    // Global metrics (4)
    vec.push_back(tns);
    vec.push_back(wns);
    vec.push_back(total_area);
    vec.push_back(total_power);
    
    // Path features (24)
    vec.insert(vec.end(), critical_path_slacks.begin(), critical_path_slacks.end());
    vec.insert(vec.end(), path_cell_counts.begin(), path_cell_counts.end());
    vec.insert(vec.end(), spare_cell_distances.begin(), spare_cell_distances.end());

    printf("Path features\n");
    printf("Number of critical path slacks %d\n", critical_path_slacks.size());
    printf("Number of path cell counts %d\n", path_cell_counts.size());
    printf("Spare cell distances %d\n", spare_cell_distances.size());
    
    /*
      TODO:
      Additional features for different move types
      // Additional features for different move types
      vec.insert(vec.end(), high_fanout_net_caps.begin(), high_fanout_net_caps.end());
      vec.insert(vec.end(), register_slack_margins.begin(), register_slack_margins.end());
      vec.insert(vec.end(), logic_depth_per_path.begin(), logic_depth_per_path.end());
      vec.insert(vec.end(), pipeline_candidates.begin(), pipeline_candidates.end());
     */
    // History features

    printf("Recent improvements (%d):\n",recent_improvements.size());
    for (auto imp: recent_improvements){
      printf("Recent improvements %f\n", imp);
    }
    
    vec.insert(vec.end(), recent_improvements.begin(), recent_improvements.end());
    printf("History features\n");
    printf("Moves attempted %f Moves accepted %f\n",
	   moves_attempted,
	   moves_accepted);
	   
    vec.push_back(static_cast<double>(moves_attempted));
    vec.push_back(static_cast<double>(moves_accepted));

    printf("State Feature size : %d\n",vec.size());
    printf("--------------\n");    
    return vec;
}

  std::string EcoAction::toString(){
    switch (type) {
    case EcoAction::ActionType::RESIZE:
      return std::string("Resize");
    default:
      return std::string("Unsupported");
    }
  }

  
} // namespace eco
