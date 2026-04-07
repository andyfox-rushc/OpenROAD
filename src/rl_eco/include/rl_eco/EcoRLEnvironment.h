#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <deque>
#include "rl_eco/EcoDesignManager.h"
#include "rl_eco/EcoTypes.h"



namespace eco {

// Forward declarations
class QNetwork;
class DQNAgent;

// ========== ECO Move Type Definitions ==========

  
// 1. Resize Action - Replace instance with different drive strength
struct ResizeAction {
  std::string instance_name;
  std::string new_master;          // e.g., BUF_X2 -> BUF_X4
  std::string spare_instance; //redundant
  
  double predicted_improvement;
  std::shared_ptr<SpareCell> spare_candidate;
  
  // Source information
  int critical_path_index;
  int instance_index_in_path;
};

// 2. Rebuffer Action - Insert/remove/modify buffers on a net
struct RebufferAction {
    enum class BufferOp {
        INSERT_BUFFER,      // Add buffer to net
        REMOVE_BUFFER,      // Remove existing buffer
        SPLIT_BUFFER,       // Split one buffer into multiple
        MERGE_BUFFERS       // Combine multiple buffers
    };
    
    BufferOp operation;
    std::string net_name;
    std::string buffer_master;       // Buffer type to insert (e.g., BUF_X2)
    
    // For INSERT_BUFFER
    double insertion_position;       // 0.0 to 1.0 along net length
    std::string spare_instance;      // Which spare to use
    
    // For REMOVE_BUFFER/MERGE_BUFFERS
    std::vector<std::string> buffers_to_modify;
    
    // Routing aware
    bool maintain_layer;             // Keep on same routing layer
    int target_layer;               
    
    double predicted_improvement;
    int critical_path_index;
};

// 3. Load Split Action - Split high fanout nets
struct LoadSplitAction {
    std::string net_name;
    std::string driver_instance;
    
    enum class SplitStrategy {
        SPATIAL,           // Split by physical location
        CRITICAL_FIRST,    // Prioritize critical sinks
        BALANCED,          // Equal fanout distribution
        LAYER_AWARE        // Consider routing layers
    };
    
    SplitStrategy strategy;
    int num_splits;                  // How many ways to split (2, 3, 4...)
    
    // Specify which sinks go to which split
    std::vector<std::vector<std::string>> sink_groups;
    
    // Buffers to drive each split
    std::vector<std::string> split_buffer_masters;
    std::vector<std::string> spare_instances;  // Spares to use
    
    // Physical clustering hints
    std::vector<double> cluster_x_centers;
    std::vector<double> cluster_y_centers;
    
    double predicted_improvement;
    double estimated_cap_reduction;  // Per split
};

// 4. Retime Action - Move sequential elements
struct RetimeAction {
    enum class RetimeOp {
        FORWARD_RETIME,     // Move register forward through logic
        BACKWARD_RETIME,    // Move register backward through logic
        DUPLICATE_REGISTER, // Clone register for high fanout
        MERGE_REGISTERS     // Combine equivalent registers
    };
    
    RetimeOp operation;
    std::string register_instance;
    
    // For FORWARD/BACKWARD_RETIME
    std::vector<std::string> logic_instances_to_cross;  // Logic to move across
    std::string target_position;     // Where to place register
    
    // For DUPLICATE_REGISTER  
    int num_duplicates;
    std::vector<std::string> sink_assignments;  // Which sinks to each copy
    std::vector<std::string> spare_registers;   // Available spare flops
    
    // For MERGE_REGISTERS
    std::vector<std::string> registers_to_merge;
    
    // Timing constraints
    bool verify_no_hold_violation;
    double min_slack_threshold;
    
    double predicted_improvement;
    int affected_paths_count;
};

// 5. Pipeline Action - Insert pipeline registers
struct PipelineAction {
    enum class PipelineStrategy {
        CRITICAL_PATH,      // Pipeline longest paths
        WAVE_PIPELINE,      // Systematic wave pipelining
        SELECTIVE,          // User-specified positions
        MIN_AREA            // Minimize register count
    };
    
    PipelineStrategy strategy;
    
    // Path or region to pipeline
    std::vector<std::string> path_instances;  // Logic path to pipeline
    std::string start_point;
    std::string end_point;
    
    // Pipeline configuration
    int num_stages;                          // How many pipeline stages
    std::vector<double> stage_positions;     // Where to cut (0.0 to 1.0)
    
    // Register selection
    std::string register_master;             // Type of register to use
    std::vector<std::string> spare_registers; // Available spares
    
    // For multi-bit paths
    int bus_width;
    bool maintain_bit_alignment;
    
    // Clock domain
    std::string clock_net;
    bool add_clock_gating;
    
    double predicted_improvement;
    double estimated_frequency_gain;
};

// 6. Logic Remap Action - Change logical function implementation
struct LogicRemapAction {
    enum class RemapType {
        FACTORIZE,         // Factor common sub-expressions
        EXPAND,            // Expand to reduce levels
        TECHNOLOGY_MAP,    // Use different cell types
        POLARITY_OPT,      // Optimize inversions
        STRENGTH_REMAP     // Change to different drive strengths
    };
    
    RemapType remap_type;
    
    // Target logic cone
    std::vector<std::string> logic_cone_instances;
    std::string cone_output;
    
    // For FACTORIZE
    std::vector<std::string> common_inputs;
    std::string factored_gate_type;         // Gate to implement factor
    
    // For EXPAND
    int max_gate_inputs;                    // Limit on gate size
    bool prefer_nand_nor;                   // Use NAND/NOR vs AND/OR
    
    // For TECHNOLOGY_MAP
    std::unordered_map<std::string, std::string> cell_mappings; // old -> new
    bool preserve_timing_critical;
    
    // For POLARITY_OPT
    bool push_inversions_forward;
    bool merge_inverter_pairs;
    
    // Spare usage
    std::vector<std::string> available_spare_gates;
    std::unordered_map<std::string, int> spare_type_count;
    
    double predicted_improvement;
    int logic_levels_delta;         // Change in logic depth
    double area_delta_estimate;
};
  


struct EcoAction {
    enum class ActionType {
        RESIZE,
        REBUFFER,
        LOAD_SPLIT,
        RETIME,
        PIPELINE,
        LOGIC_REMAP
    };
    
    ActionType type;
    
    // Union-like structure (in practice might use std::variant)
    std::unique_ptr<ResizeAction> resize;
    std::unique_ptr<RebufferAction> rebuffer;
    std::unique_ptr<LoadSplitAction> load_split;
    std::unique_ptr<RetimeAction> retime;
    std::unique_ptr<PipelineAction> pipeline;
    std::unique_ptr<LogicRemapAction> logic_remap;
    
    // Common fields
  double predicted_improvement;
  double confidence_score;
  int affected_critical_paths;
  std::string toString();
};
  
// State representation remains the same...
struct DesignState {
    // Global metrics
    double tns;                    
    double wns;                    
    double total_area;
    double total_power;
    
    // Path-specific features
    std::vector<double> critical_path_slacks;     
    std::vector<double> path_cell_counts;         
    std::vector<double> spare_cell_distances;     
    
    // Additional features for different move types
    std::vector<double> high_fanout_net_caps;      // For load split
    std::vector<double> register_slack_margins;    // For retiming
    std::vector<double> logic_depth_per_path;      // For logic remap
    std::vector<double> pipeline_candidates;       // For pipelining
    
    // Move history features
    std::vector<double> recent_improvements;      
    int moves_attempted;
    int moves_accepted;
    
    // Move type distribution
    std::unordered_map<EcoAction::ActionType, int> moves_by_type;
  //get design state as vector
    std::vector<double> toVector() const;
};

  
class EcoRLEnvironment {
  const unsigned SHANNON_DEPTH=4;
public:
  EcoRLEnvironment(std::shared_ptr<EcoDesignManager> manager, 
                     rsz::Resizer* resizer,
                     int num_critical_paths = 10,
                     int max_moves_per_episode = 100);
    
    ~EcoRLEnvironment();
    
    // Episode management
    void reset();
    bool isEpisodeDone() const;

  //Is this path shannon split feasible ?
  bool IsShannonFeasible(const eco::PathInfo& path_info,
			 odb::dbITerm* &start_iterm,
			 odb::dbITerm* &end_iterm,
			 unsigned N);
  
  size_t state_size() {return state_size_;}
  void state_size(size_t s){state_size_ = s;}
  
  // Core RL interface - now handles any action type
  DesignState getCurrentState();
  double step(const std::shared_ptr<EcoAction> action);  // Returns reward

  //Agent interface for actions
  size_t getActionSize();
  size_t getIndexFromAction(std::shared_ptr<EcoAction>);
  std::shared_ptr<EcoAction> getActionFromIndex(size_t index);
  std::vector<size_t> getValidActionIndices(const DesignState& state);


  
  size_t getStateSize();
  
    // Action space management for different types
  std::vector<std::shared_ptr<EcoAction> > getValidActions(const DesignState& state);
    
  // Specific action generators
  std::vector<std::shared_ptr<EcoAction> > generateResizeActions(const PathAnalysis& analysis);
  std::vector<std::shared_ptr<EcoAction> > generateShannonActions(const PathAnalysis& analysis);
  std::vector<std::shared_ptr<EcoAction> > generateRebufferActions(const PathAnalysis& analysis);
  std::vector<std::shared_ptr<EcoAction> > generateLoadSplitActions(const PathAnalysis& analysis);
  std::vector<std::shared_ptr<EcoAction> > generateRetimeActions(const PathAnalysis& analysis);
  std::vector<std::shared_ptr<EcoAction> > generatePipelineActions(const PathAnalysis& analysis);
  std::vector<std::shared_ptr<EcoAction> > generateLogicRemapActions(const PathAnalysis& analysis);
     
    
    // Metrics for training monitoring
    struct EpisodeStats {
        double initial_tns;
        double final_tns;
        int moves_accepted;
        int moves_rejected;
        double total_reward;
        std::vector<double> tns_trajectory;
    };

  
  EpisodeStats getEpisodeStats() const { return current_stats_; }

  EcoDesignManager::MoveResult executeMove(const std::shared_ptr<EcoAction> action);  
  size_t max_action_size(){return max_action_size_;}

private:
    // Core components
  std::shared_ptr<EcoDesignManager> design_manager_;
  rsz::Resizer* resizer_;
    
    // Configuration
    const int num_critical_paths_;
    const int max_moves_per_episode_;
    const double min_improvement_threshold_ = 0.1;  // ps
  static constexpr size_t MAX_ACTIONS = 500;  // Fixed upper bound

    // Episode state
    int current_move_;
    bool episode_active_;
    EpisodeStats current_stats_;
    std::deque<double> recent_improvements_;  // Sliding window
  size_t max_action_size_=0;
  size_t state_size_=0;
  PathAnalysis analyzeCriticalPaths();
    
    // Action generation
  double estimateActionValue(const std::shared_ptr<EcoAction>& action,
			     const PathAnalysis& analysis);
    
    // Reward calculation
  double calculateReward(const EcoDesignManager::MoveResult& result, 
			 double prev_tns, 
			 double prev_wns);
    
    // Move execution with journaling

  void acceptMove(const std::shared_ptr<EcoAction> action);
  void rejectMove(const std::shared_ptr<EcoAction> action);
    
    // State extraction helpers
    std::vector<double> extractPathFeatures(const std::vector<PathInfo>& paths);
    std::vector<double> extractSpareUtilization();
    
    // Utilities
    double calculateManhattanDistance(const std::string& inst1, 
                                    const std::string& inst2);
  std::vector<std::shared_ptr<SpareCell> > findNearbySpares(odb::dbInst*,
					    long long radius);


  std::map<size_t,std::shared_ptr<EcoAction> > index2action_;
  std::map<std::shared_ptr<EcoAction>,size_t > action2index_;
  
  //as we commit moves we keep track of the spare cells we have used in execute move
  //in any state.
  std::set<std::shared_ptr<SpareCell> > all_states_used_spare_cells_;
  
  friend class EcoQLearningTrainer; //to access resizer
};

} // namespace eco
