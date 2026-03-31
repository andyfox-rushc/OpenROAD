#!/usr/bin/env openroad
cd ~/rl_test/adder_reduce
source ~/OpenROAD_rl/src/rl_eco/src/RlEco.tcl
info commands eco*
puts "================================================"
puts "Testing RL-based Resizer"
puts "================================================\n"

read_lef NangateOpenCellLibrary.tech.lef
read_lef NangateOpenCellLibrary.macro.mod.lef
read_def placed_with_spares.def
read_liberty NangateOpenCellLibrary_typical.lib
read_sdc constraint.sdc
report_checks


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
#sta::update_timing
set initial_wns [sta::worst_slack -max]
set initial_tns [sta::total_negative_slack -max]
puts "Initial WNS: ${initial_wns}ns"
puts "Initial TNS: ${initial_tns}ns"


# Initialize RL-ECO
puts "\n=== Initializing RL-ECO Environment ==="

# Initialize the RL ECO module (assuming it's loaded)
# This creates the C++ objects needed
#eco_init

set eco [eco::getRlEco]
$eco reportFeatures


# Identify spare cells - critical for ECO
$eco identifySpareCells
$eco reportSpareCells
set vec [eco::IntVector]
#eco::IntVector_push $vec 256
#eco::IntVector_push $vec 128
#eco::IntVector_push $vec 64

eco::IntVector_push $vec 256
eco::IntVector_push $vec 256
eco::IntVector_push $vec 256
#eco::IntVector_push $vec 256

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
set episodes 20
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
$eco reportFeatures    
# Run training
$eco trainAgent $episodes $learning_rate $epsilon $gamma
$eco saveModel "$model_dir/model.model"
$eco reportFeatures

$eco applyTrainedModel

