# scripts/rl_eco_generation_test.tcl
# RL-based ECO generation with hardcoded design files
source ~/OpenROAD_rl/src/rl_eco/src/RlEco.tcl
info commands eco*
puts "================================================"
puts "RL-based ECO Generation for Timing Optimization"
puts "================================================"

read_lef NangateOpenCellLibrary.tech.lef
read_lef NangateOpenCellLibrary.macro.mod.lef
read_def placed_with_spares.def
read_liberty NangateOpenCellLibrary_typical.lib
read_sdc constraint.sdc


# Report initial design stats
set block [ord::get_db_block]
set insts [llength [$block getInsts]]
set nets [llength [$block getNets]]
puts "Design statistics:"
puts "  Instances: $insts"
puts "  Nets: $nets"

# Initialize timing analysis
puts "\n=== Initial Timing Analysis ==="
estimate_parasitics -placement
report_worst_slack -max
report_worst_slack -min

# Get initial timing metrics
sta::update_timing
set initial_wns [sta::worst_slack -max]
set initial_tns [sta::total_negative_slack -max]
puts "Initial WNS: ${initial_wns}ns"
puts "Initial TNS: ${initial_tns}ns"


# Initialize RL-ECO
puts "\n=== Initializing RL-ECO Environment ==="

# Initialize the RL ECO module (assuming it's loaded)
# This creates the C++ objects needed
eco_init

set eco [eco::getRlEco]
# Identify spare cells - critical for ECO
puts "\n=== Identifying Spare Cells ==="
$eco identifySpareCells

# Report spare cells found
set num_spares [eco_get_num_spare_cells]
puts "Found $num_spares spare cells"
puts "Report on distribution"
$eco reportSpareCells


# Configure the unified environment for generation mode
puts "\n=== Configuring RL Environment ==="
$eco setUnifiedMode "generation"

# Set RL hyperparameters
set vec [eco::IntVector]
eco::IntVector_push $vec 256
eco::IntVector_push $vec 128
eco::IntVector_push $vec 64
$eco setHiddenLayers $vec

$eco setBatchSize 32
$eco setReplayBufferSize 10000
$eco setRewardWeights 1.0 0.3 0.2

# Model file path
set model_dir "./rl_eco_models"
file mkdir $model_dir
set model_file "$model_dir/eco_generation_model.ckpt"
    
# Train the RL agent
#episodes was 200
set episodes 2
set learning_rate 0.001
set epsilon 0.3      ;# exploration rate
set gamma 0.99       ;# discount factor
    
puts "Training parameters:"
puts "  Episodes: $episodes"
puts "  Learning rate: $learning_rate"
puts "  Epsilon: $epsilon"
puts "  Gamma: $gamma"
    
# Enable checkpointing during training
$eco enableCheckpoints "$model_dir/checkpoints" 50
    
# Run training
$eco trainUnified $episodes $learning_rate $epsilon $gamma
set eco_file "timing_fixes.eco"
$eco writeUnifiedEcoFile $eco_file
puts "Generated ECO file $eco_file"


# Save the trained model
$eco saveModel $model_file
puts "\nSaved trained model to: $model_file"


###### rest
puts "\n=== Configuring RL Environment ==="

$eco identifySpareCells
$eco reportSpareCells

#$eco loadEcoChanges "timing_fixes.eco"

$eco setUnifiedMode "implementation"
$eco identifySpareCells
$eco reportSpareCells
$eco runUnifiedImplementation
