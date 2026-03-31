cd ~/rl_test/adder_redce
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
set model_dir "./rl_eco_models"

#set vec [eco::IntVector]
#eco::IntVector_push $vec 256
#eco::IntVector_push $vec 256
#eco::IntVector_push $vec 256
#eco::IntVector_push $vec 256
#$eco setHiddenLayers $vec
#35
$eco loadModel "$model_dir/model.model" 324 500
$eco applyTrainedModel

puts "\n=== Final Timing Analysis ==="
estimate_parasitics -placement
report_worst_slack -max
report_worst_slack -min
