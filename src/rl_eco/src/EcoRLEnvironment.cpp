#include "ord/OpenRoad.hh"
#include "rl_eco/EcoTypes.h" 
#include "rl_eco/EcoRLEnvironment.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <chrono>
#include <set>
#include <map>
#include <string>
#include <iostream>
#include <sstream>

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
      //make the spare cell as used
      acceptMove(action);
      printf("Accepted move gain %.10f and not restored state, but ended journal\n",
	     result.timing_improvement);
        current_stats_.moves_accepted++;
        recent_improvements_.push_back(result.timing_improvement);
    } else {
      printf("Rejected move and not restored state, ended journal\n");      
        rejectMove(action);
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
    printf("Number of valid action indices %d\n", valid_indices.size());
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

  // Generate actions of each type  
  //  printf("Generating Shannon Actions\n");
  std::vector<std::shared_ptr<EcoAction> > shannon_actions = generateShannonActions(analysis);
  
  

  start = std::chrono::steady_clock::now();    
  auto resize_actions = generateResizeActions(analysis);
  end = std::chrono::steady_clock::now();
  duration = end-start;


    auto rebuffer_actions = generateRebufferActions(analysis);
    auto load_split_actions = generateLoadSplitActions(analysis);
    auto retime_actions = generateRetimeActions();
    auto pipeline_actions = generatePipelineActions(analysis);
    auto logic_remap_actions = generateLogicRemapActions(analysis);

    // Combine candidates to form all actions    
    std::vector<std::shared_ptr<EcoAction>> all_actions;    
    printf("# %d shannon actions\n", shannon_actions.size());
    all_actions.insert(all_actions.end(), shannon_actions.begin(), shannon_actions.end());
    printf("# %d resize actions\n", resize_actions.size());
    all_actions.insert(all_actions.end(), resize_actions.begin(), resize_actions.end());
    all_actions.insert(all_actions.end(), rebuffer_actions.begin(), rebuffer_actions.end());
    all_actions.insert(all_actions.end(), load_split_actions.begin(), load_split_actions.end());
    //    all_actions.insert(all_actions.end(), retime_actions.begin(), retime_actions.end());
    all_actions.insert(all_actions.end(), pipeline_actions.begin(), pipeline_actions.end());
    all_actions.insert(all_actions.end(), logic_remap_actions.begin(), logic_remap_actions.end());


    printf("Total Number of actions %d\n", all_actions.size());
    
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


  void EcoRLEnvironment::populateCompatibleSpareCells(odb::dbMaster* master_in,
						      std::map<odb::dbMaster*,
						      std::vector<std::shared_ptr<SpareCell>> >
						      &master2candidates){

    std::set<std::string> compatible_masters = design_manager_ ->
      getCompatibleMasters(master_in -> getName());

    for (auto s: compatible_masters){

      if (master2candidates.find(master_in) == master2candidates.end()){
	std::vector<std::shared_ptr<SpareCell> > spare_vec;
	
	//checks that spares are not used.
	std::vector<std::shared_ptr<SpareCell> > spare_cells =
	  design_manager_ ->
	  getAvailableSpares(s);

	if (spare_cells.size() > 0){
	  for (auto s: spare_cells){
	    spare_vec.push_back(s);
	  }
	  master2candidates[master_in]=spare_vec;
	}
      }
    }
  }


  /*
    The Shannon gates to duplicate look something like:

    INST{start_iterm} -> G1 -> G2 -> G3{end_iterm}

    We do not copy INST. Instead we fish out its output (start_iterm) and use
    that as the "cut point".
    
  */
  
						      
  bool EcoRLEnvironment::IsShannonFeasible(const eco::PathInfo& path_info,

					   odb::dbITerm* &start_iterm, //an output pin. Drives first
					   //gate in chain
					   odb::dbITerm* &end_iterm,   //an output pin. Driven by
					   //last gate in chain

					   //
					   //The gates in the chain to copy
					   //Note we don't copy the one driving start_iterm.
					   //That is our "cut point"
					   //
					   std::vector<std::pair<odb::dbInst*,
					   std::shared_ptr<SpareCell> > >&
					   gates_to_duplicate,
					   int N //how deep our chain should be.
					   ){

    std::vector<std::map<odb::dbInst*, std::shared_ptr<SpareCell> > >   path_mapping;
    
    std::map<odb::dbMaster*, std::vector<std::shared_ptr<SpareCell> >  >    master2candidates;
    std::vector<std::pair<int,int>> partitions;

    std::vector<std::shared_ptr<SpareCell> > spare_cells_marked_as_used;
    int start_ix = -1;
    int end_ix = -1;
    int max_partition_size = 0;
    int partition_ix = 0;

    path_mapping.resize(path_info.instances.size());
    
    /*
    printf("Examining path:\n");
    for (int inst_idx = 0; inst_idx < path_info.instances.size(); ++inst_idx) {
      odb::dbInst* inst = path_info.instances[inst_idx];
      printf("%s (%s) -> ",
	     inst -> getName().c_str(),
	     inst -> getMaster()-> getName().c_str());
    }
    printf("\n");
    */
    
    //first populate the equivalent spare cells per master used in the path

    for (int inst_idx = 0; inst_idx < path_info.instances.size(); ++inst_idx) {
      odb::dbInst* inst = path_info.instances[inst_idx];
      odb::dbMaster* current_master = inst -> getMaster();

      populateCompatibleSpareCells(current_master, master2candidates);
      
      std::vector<std::shared_ptr<SpareCell> > &candidates =
	master2candidates[inst -> getMaster()];

      long long  max_radius_squared = design_manager_ -> getMaxRadius(inst);
      
      // Get instance location (center point)      
      int inst_x;
      int inst_y;
      inst->getLocation(inst_x, inst_y);
      // Get instance bounding box to find center
      odb::dbBox* bbox = inst->getBBox();
      if (bbox) {
        inst_x = (bbox->xMin() + bbox->xMax()) / 2;
        inst_y = (bbox->yMin() + bbox->yMax()) / 2;
      }
      bool no_fit = true;
      std::shared_ptr<SpareCell> candidate_spare_cell = nullptr;
      if (candidates.size() != 0){
	bool found_candidate =false;
	for (auto s: candidates){
	  if (s -> is_used == false){
	    odb::dbInst* spare_inst = s -> instance;
	    if (!spare_inst){
	      continue;
	    }
	    int spare_x, spare_y;
	    spare_inst->getLocation(spare_x, spare_y);
	    // Get spare cell center
	    odb::dbBox* spare_bbox = spare_inst->getBBox();
	    if (spare_bbox) {
	      spare_x = (spare_bbox->xMin() + spare_bbox->xMax()) / 2;
	      spare_y = (spare_bbox->yMin() + spare_bbox->yMax()) / 2;
	    }
	    long long dx = static_cast<long long>(spare_x - inst_x);
	    long long dy = static_cast<long long>(spare_y - inst_y);
	    long long dist_squared = dx * dx + dy * dy;
	    /*
	    printf("Dist squared %ld Max Radius %ld Inst %s Spare Inst %s \n",
		   dist_squared,
		   max_radius_squared,
		   inst -> getName().c_str(),
		   spare_inst -> getName().c_str()
		   );
	    */
	    if (dist_squared <= static_cast<long long>(max_radius_squared)) {
	      s-> is_used= true;
	      found_candidate = true;
	      spare_cells_marked_as_used.push_back(s);
	      if (start_ix == -1){
		start_ix = inst_idx;
	      }
	      end_ix = inst_idx;
	      path_mapping[partition_ix][inst] = s;
	      no_fit = false;
	      break; //found a mapping
	    }
	    else{
	      //no fit
	      continue;
	    }
	  }
	}
      }
      
      
      if (no_fit){
	//a break
	//record non trivial partitions
	if (start_ix != -1 && end_ix !=-1){
	  partitions.push_back(std::pair<int,int>(start_ix,end_ix));
	  max_partition_size = end_ix - start_ix;
	  partition_ix++;

	  {
	    /*
	    printf("Created partition:\n");
	    for (int i= start_ix; i <= end_ix; i++){
	      odb::dbInst* local_inst = path_info.instances[i];
	      std::shared_ptr<SpareCell> local_spare =  path_mapping[partition_ix-1][local_inst];
	      printf("Inst %s . Spare %s\n",
		     local_inst -> getName().c_str(),
		     local_spare ?
		     local_spare ->  instance -> getName().c_str():
		     "unknown-spare "
		     );
	      
	    }
	    */
	  }
	  
	}
	start_ix = -1;
	end_ix = -1;
      }
    }

    bool feasible_split = false;
    partition_ix=0;
    if (max_partition_size >= N) {
      for (auto p: partitions){
	if (p.second - p.first == max_partition_size){
	  //start_iterm is output of first gate
	  //construct the return gates
	  /*  printf("First %d (%s) Last %d (%s)\n",
		 p.first,
		 path_info.instances[p.first] -> getName().c_str(),
		 p.second,
		 path_info.instances[p.second] -> getName().c_str()
		 );
	  */
	  for (	  int local_ix = p.first;
		  local_ix <= p.second;
		  local_ix=local_ix+1){
	    odb::dbInst* inst = path_info.instances[local_ix];


	    if (local_ix == p.first){
	      start_iterm = inst -> getFirstOutput();
	      auto spare = path_mapping[partition_ix][inst];
	      //we skip the first cell, we only use its output
	      spare -> is_used = false;
	    }
	    
	    if (local_ix == p.second){
	      end_iterm = inst -> getFirstOutput();
	    }
	    std::shared_ptr<SpareCell> spare;
	    if (path_mapping[partition_ix].find(inst) ==
		path_mapping[partition_ix].end()){
	      /*
	      printf("Inst %s has no spare. local_ix %d\n",
		     inst -> getName().c_str(),
		     local_ix);	      
	      */
	    }

	    //only copy the following cells.
	    if (local_ix != p.first){
	      spare = path_mapping[partition_ix][inst];
	      assert(spare);
	      gates_to_duplicate.push_back(
					   std::pair<odb::dbInst*,
					   std::shared_ptr<SpareCell> >(inst,spare)
					   );
	    }
	  }
	  feasible_split=true;
	  break;
	}
	partition_ix++;
      }
    }
    return feasible_split;
  }


  
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateShannonActions(
					 const PathAnalysis& analysis)
{
  std::vector<std::shared_ptr<EcoAction>> actions;

  
  for (int path_idx = 0; path_idx < analysis.critical_paths.size(); ++path_idx) {
        const auto& path = analysis.critical_paths[path_idx];
	odb::dbITerm* start_iterm=nullptr;
	odb::dbITerm* end_iterm=nullptr;
	std::vector<std::pair<odb::dbInst*,std::shared_ptr<SpareCell>> > gates_to_duplicate;
	odb::dbITerm* mux2xn_d0=nullptr;
	odb::dbITerm* mux2xn_d1=nullptr;
	odb::dbITerm* mux2xn_sel=nullptr;
	odb::dbITerm* mux2xn_op=nullptr;
	
	std::shared_ptr<SpareCell> spare_mux_cell;	
	if (design_manager_ -> getSpareMux2(spare_mux_cell,
					    mux2xn_d0,
					    mux2xn_d1,
					    mux2xn_sel,
					    mux2xn_op) &&
	    IsShannonFeasible(path,
			      start_iterm, //output pin of gate driving 1st member of chain
			      end_iterm,   //output pin of last in chain
			      gates_to_duplicate,
			      EcoRLEnvironment::SHANNON_DEPTH)){

	  EcoDesignManager::MoveResult result =
	    design_manager_ -> previewPathSplit(
						start_iterm,
						end_iterm,
						spare_mux_cell,
						mux2xn_d0,
						mux2xn_d1,
						mux2xn_sel,
						mux2xn_op,
						gates_to_duplicate
						);
	  if (result.timing_improvement > 0) {
	    
	    auto action = std::make_shared<EcoAction>();
	    action-> path_split = std::make_unique<PathSplitAction>();	  
	    action -> type = EcoAction::ActionType::PATH_SPLIT;
	    action -> path_split -> start_point = start_iterm;
	    action -> path_split -> end_point = end_iterm;
	    //deep copy
	    action -> path_split -> gates_to_duplicate = gates_to_duplicate;
	    action -> path_split -> shannon_depth = gates_to_duplicate.size();
	    action-> path_split ->predicted_improvement = result.timing_improvement;
	    action -> path_split -> spare_mux_cell = spare_mux_cell;
	    action -> path_split -> mux2xn_d0 = mux2xn_d0;
	    action -> path_split -> mux2xn_d1 = mux2xn_d1;
	    action -> path_split -> mux2xn_sel = mux2xn_sel;
	    action -> path_split -> mux2xn_op = mux2xn_op;	    
	    
	    actions.push_back(action);
	  }
	}
  }
  return actions;
}

  
std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateResizeActions(
										const PathAnalysis& analysis) {

  //use the caching
  design_manager_ -> wireLengthCache(true);
  
  std::vector<std::shared_ptr<EcoAction>> actions;
  std::map<std::string,std::set<std::string> > compatible_masters_set; //shared across all runs
  std::map<std::string,std::set<std::string> > ::iterator  compatible_masters_set_it;
  std::set<std::string> compatible_masters;
    
  std::set<std::shared_ptr<SpareCell> > used_spare_cells;
    
    // For each critical path
    for (int path_idx = 0; path_idx < analysis.critical_paths.size(); ++path_idx) {
        const auto& path = analysis.critical_paths[path_idx];
	sta::dbSta* sta = ord::OpenRoad::openRoad()->getSta();

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


	    auto start = std::chrono::steady_clock::now();

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

	    for (auto spare_candidate: nearby_spares){
	      //skip any spares we have already used.
	      if (used_spare_cells.find(spare_candidate) != used_spare_cells.end()){
		continue;
	      }

	      if (all_states_used_spare_cells_.find(spare_candidate) !=
		  all_states_used_spare_cells_.end()){
		continue;
	      }
	      
	      odb::dbInst* spare_inst = spare_candidate -> instance;
	      odb::dbMaster* spare_master = spare_candidate -> master;
	      std::string spare_master_name = spare_master -> getName();

	      //Is the candidate spare compatible with the master ?
	      bool compatible = false;
	      //	      printf("master for candidate spare %s\n", spare_master_name.c_str());
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
	      
	      //Preview the move, will it lead to a good result ?
	      EcoDesignManager::MoveResult move_gain = 
		design_manager_->previewResize(inst, spare_inst);

		auto end = std::chrono::steady_clock::now();
		duration = end-start;

	      // Only create action if there's predicted improvement
                if (move_gain.timing_improvement > 0) {
		  //yes form the action
                    // Create EcoAction with ResizeAction
                    auto action = std::make_shared<EcoAction>();
                    action->type = EcoAction::ActionType::RESIZE;
                    action->resize = std::make_unique<ResizeAction>();
                    action->resize->instance_name = instance_name;
                    action->resize->new_master = spare_master_name;
                    action->resize->critical_path_index = path_idx;
                    action->resize->instance_index_in_path = inst_idx;
                    action->resize->spare_instance = spare_candidate -> instance -> getName(); 
		    action -> resize -> spare_candidate = spare_candidate;
                    // Set metrics
                    action->predicted_improvement = move_gain.timing_improvement;
                    action->resize->predicted_improvement = action->predicted_improvement;
                    action->affected_critical_paths = 1;
                    action->confidence_score = 0.8;
                    actions.push_back(action);

		    //mark that we have used this spare cell
		    used_spare_cells.insert(spare_candidate);
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
	printf("Estimate resize action value %.10f\n",value);		
	break;
      }
    }
      
    case  EcoAction::ActionType::PATH_SPLIT: {
      if (action -> path_split){
	value = action -> predicted_improvement;
	printf("Estimate path split action value %.10f\n",value);	
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

  
  std::vector<std::shared_ptr<EcoAction>> EcoRLEnvironment::generateRetimeActions(){

    std::vector<std::shared_ptr<EcoAction>> actions;

    std::vector<std::tuple<odb::dbInst*,
			   odb::dbITerm*, //d input
			   odb::dbITerm*, //q output
			   bool  //direction of the move
			   //TODO: spare cells for the move.
			   > > retime_moves;

    design_manager_-> updateTiming(true); //do a full update timing
    design_manager_ -> identifyRetimingMoves(retime_moves);
    printf("Estimated number of retiming opportunities: %d\n", retime_moves.size());

    //generate the eco actions for the each retiming move.
    for (auto retime_move: retime_moves){
      auto action = std::make_shared<EcoAction>();
      action -> retime = std::make_unique<RetimeAction>();
      action -> type = EcoAction::ActionType::RETIME;
      odb::dbInst* flop_inst = std::get<0>(retime_move);
      odb::dbITerm* d = std::get<1>(retime_move);
      odb::dbITerm* q = std::get<2>(retime_move);
      bool forward = std::get<3>(retime_move);

      if (forward){
	action -> retime -> operation = RetimeAction::RetimeOp::FORWARD_RETIME;
      }
      else{
	action -> retime -> operation = RetimeAction::RetimeOp::BACKWARD_RETIME;	
      }
      action -> retime -> flop = flop_inst;
      action -> retime -> d = d;
      action -> retime -> q = q;

      actions.push_back(action);
    }
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
	printf("Executing resize !\n");
	std::string original_instance_name = action -> resize -> instance_name;
	std::string spare_instance_name = action -> resize -> spare_instance;
	odb::dbInst* original_instance = design_manager_ -> findInst(original_instance_name);
	odb::dbInst* new_instance = design_manager_ -> findInst(spare_instance_name);

	//mark spare instance as used.
	//if original instance is spre, mark it as free
	//TODO make this a set
	printf("Todo mark this as used %s\n",
	       spare_instance_name.c_str());
	
	return design_manager_->performInstanceSwap(original_instance,
						new_instance);
      }
      break;
    case EcoAction::ActionType::PATH_SPLIT:
      {
	printf("Executing path split !\n");
	odb::dbITerm* start = action -> path_split -> start_point;
	odb::dbITerm* end = action -> path_split -> end_point;
	std::shared_ptr<SpareCell> spare_mux_cell = action -> path_split -> spare_mux_cell;
	odb::dbITerm* mux2xn_d0 = action -> path_split -> mux2xn_d0;
	odb::dbITerm* mux2xn_d1 = action -> path_split -> mux2xn_d1;
	odb::dbITerm* mux2xn_sel = action -> path_split -> mux2xn_sel;
	odb::dbITerm* mux2xn_op=action -> path_split -> mux2xn_op;
	std::vector<std::pair<odb::dbInst*,std::shared_ptr<SpareCell>> > gates_to_duplicate
	  = action -> path_split -> gates_to_duplicate;
	
	return design_manager_ -> performPathSplit(start,
						   end,
						   spare_mux_cell,
						   mux2xn_d0,
						   mux2xn_d1,
						   mux2xn_sel,
						   mux2xn_op,
						   gates_to_duplicate);
      }
      
    default:
      break;
    }
    return default_error;
  }
  

  void EcoRLEnvironment::acceptMove(const std::shared_ptr<EcoAction> action) {

    
    if (action -> type == EcoAction::ActionType::RESIZE){
      //update the spare cell free list
      const std::shared_ptr<SpareCell> used_spare = action -> resize -> spare_candidate;
      all_states_used_spare_cells_.insert(used_spare);
    }

    if (action -> type == EcoAction::ActionType::PATH_SPLIT){
      printf("Updated path split move !\n");
      //update the spare cell free list
    }

    
  
    resizer_->journalEnd();
}

  void EcoRLEnvironment::rejectMove(const std::shared_ptr<EcoAction> action) {
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
    std::stringstream strstr;
    switch (type) {
    case EcoAction::ActionType::RESIZE:
      strstr << "Resize. Instance " << resize -> instance_name << " New master " << resize -> new_master << "Spare instance " << resize -> spare_instance << " improvements " << resize -> predicted_improvement << "\n";
      return strstr.str();
      break;
      
    case EcoAction::ActionType::PATH_SPLIT:
      if (path_split -> start_point &&
	  path_split -> end_point 
	  ){
      strstr << "Path Split cut point: " << path_split -> start_point -> getName('/') << " to  end point " << path_split -> end_point -> getName('/') << "\n";
      strstr << "Estimated gain : " << path_split -> predicted_improvement << "\n";
      strstr << "Path length: " << path_split -> shannon_depth << "\n";
      strstr << "Instances in Shannon split path:\n";
      for (auto p: path_split -> gates_to_duplicate){
	if (p.first && p.second){
	strstr << "\n Inst " << p.first -> getName() << "( " 
	       << p.first -> getMaster() -> getName() << ")" 
	       << "\tSpare " << p.second -> instance -> getName() 
	       << "( " << p.second -> master -> getName() <<" )\n";
	}
	else {
	  if (p.first == nullptr)
	    printf("p.first null\n");
	  if (p.second == nullptr)
	    printf("p.second null\n");
	  
	  strstr << "Badly formed inst/spare combination\n";
	}
      }
      strstr << "\n";
      }
      else{
	strstr << "Badly formed path\n";
      }
      return strstr.str();
      break;
    default:
      return std::string("Unsupported");
    }
  }

  
} // namespace eco
