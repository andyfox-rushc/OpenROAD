#info commands eco::*info commands eco*
puts "Loading RlEco.tcl"

sta::define_cmd_args "eco_init" {}

proc eco_init { args } {
    sta::parse_key_args "eco_init" args keys {} flags {}
    set eco [eco::getRlEco]
    puts "Pointer to eco $eco"
    puts "Getting openroad"
    set openroad [$eco getOpenRoad]
    $eco init $openroad
}

sta::define_cmd_args "eco_train" {
    [-episodes episodes]
    [-learning_rate learning_rate]
    [-epsilon epsilon]
    [-gamma gamma]
}

proc eco_train { args } {
    sta::parse_key_args "eco_train" args \
        keys {-episodes -learning_rate -epsilon -gamma} \
        flags {}

    set eco [eco::getRlEco]

    # Set defaults
    set episodes [expr {[info exists keys(-episodes)] ? $keys(-episodes) : 1000}]
    set learning_rate [expr {[info exists keys(-learning_rate)] ? $keys(-learning_rate) : 0.001}]
    set epsilon [expr {[info exists keys(-epsilon)] ? $keys(-epsilon) : 0.1}]
    set gamma [expr {[info exists keys(-gamma)] ? $keys(-gamma) : 0.99}]

    $eco trainAgent $episodes $learning_rate $epsilon $gamma
}

sta::define_cmd_args "eco_load_model" {model_path}

proc eco_load_model { args } {
    sta::parse_key_args "eco_load_model" args keys {model_path} flags {}
    
    set eco [eco::getRlEco]
    $eco loadModel $keys(model_path)
}

sta::define_cmd_args "eco_save_model" {model_path}

proc eco_save_model { args } {
    sta::parse_key_args "eco_save_model" args keys {model_path} flags {}
    
    set eco [eco::getRlEco]
    $eco saveModel $keys(model_path)
}

sta::define_cmd_args "eco_set_hidden_layers" {layers}

proc eco_set_hidden_layers { args } {
    sta::parse_key_args "eco_set_hidden_layers" args keys {layers} flags {}
    
    set eco [eco::getRlEco]
    
    # Convert TCL list to vector
    set layer_list [split $keys(layers) ","]
    
    # Create IntVector and add elements
    set vec [new_IntVector]
    foreach layer $layer_list {
        IntVector_push_back $vec $layer
    }
    
    $eco setHiddenLayers $vec
    
    # Clean up
    delete_IntVector $vec
}


sta::define_cmd_args "eco_apply_rl" {}

proc eco_apply_rl { args } {
    sta::parse_key_args "eco_apply_rl" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco applyEcoWithRL
}

sta::define_cmd_args "eco_apply_greedy" {}

proc eco_apply_greedy { args } {
    sta::parse_key_args "eco_apply_greedy" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco applyEcoGreedy
}

sta::define_cmd_args "eco_set_reward_weights" {
    timing_weight
    wirelength_weight
    congestion_weight
}

proc eco_set_reward_weights { args } {
    sta::parse_key_args "eco_set_reward_weights" args \
        keys {timing_weight wirelength_weight congestion_weight} \
        flags {}
    
    set eco [eco::getRlEco]
    $eco setRewardWeights $keys(timing_weight) $keys(wirelength_weight) $keys(congestion_weight)
}

sta::define_cmd_args "eco_report" {}

proc eco_report { args } {
    sta::parse_key_args "eco_report" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco reportMetrics
}

sta::define_cmd_args "eco_clear_changes" {}

proc eco_clear_changes { args } {
    sta::parse_key_args "eco_clear_changes" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco clearEcoChanges
}

sta::define_cmd_args "eco_add_spare_pattern" {pattern}

proc eco_add_spare_pattern { args } {
    sta::parse_key_args "eco_add_spare_pattern" args keys {pattern} flags {}
    
    set eco [eco::getRlEco]
    $eco addSparePattern $keys(pattern)
}

sta::define_cmd_args "eco_enable_checkpoints" {directory interval}

proc eco_enable_checkpoints { args } {
    sta::parse_key_args "eco_enable_checkpoints" args \
        keys {directory interval} flags {}
    
    set eco [eco::getRlEco]
    $eco enableCheckpoints $keys(directory) $keys(interval)
}

sta::define_cmd_args "eco_disable_checkpoints" {}

proc eco_disable_checkpoints { args } {
    sta::parse_key_args "eco_disable_checkpoints" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco disableCheckpoints
}

sta::define_cmd_args "eco_set_batch_size" {size}

proc eco_set_batch_size { args } {
    sta::parse_key_args "eco_set_batch_size" args keys {size} flags {}
    
    set eco [eco::getRlEco]
    $eco setBatchSize $keys(size)
}

sta::define_cmd_args "eco_set_replay_buffer_size" {size}

proc eco_set_replay_buffer_size { args } {
    sta::parse_key_args "eco_set_replay_buffer_size" args keys {size} flags {}
    
    set eco [eco::getRlEco]
    $eco setReplayBufferSize $keys(size)
}


sta::define_cmd_args "eco_load_changes" {filename}
 
proc eco_load_changes { args } {
    if { [llength $args] != 1 } {
        utl::error ECO 1 "eco_load_changes requires exactly one argument: filename"
    }
    
    set filename [lindex $args 0]
    
    set eco [eco::getRlEco]
    $eco loadEcoChanges $filename
}
 
sta::define_cmd_args "eco_identify_spare" {}
 
proc eco_identify_spare { args } {
    set eco [eco::getRlEco]
    $eco identifySpareCells
}
 
sta::define_cmd_args "eco_apply_simple" {}
 
proc eco_apply_simple { args } {
    set eco [eco::getRlEco]
    $eco applySimpleEco
}
 
sta::define_cmd_args "eco_write_def" {filename}
 
proc eco_write_def { args } {
    if { [llength $args] != 1 } {
        utl::error ECO 1 "eco_write_def requires exactly one argument: filename"
    }
    
    set filename [lindex $args 0]
    
    set eco [eco::getRlEco]
    $eco writeDEF $filename
}
 

sta::define_cmd_args "eco_clear_changes" {}
 
proc eco_clear_changes { args } {
    set eco [eco::getRlEco]
    $eco clearEcoChanges
}
 
sta::define_cmd_args "eco_add_spare_pattern" {pattern}
 
proc eco_add_spare_pattern { args } {
    if { [llength $args] != 1 } {
        utl::error ECO 2 "eco_add_spare_pattern requires exactly one argument: pattern"
    }
    
    set pattern [lindex $args 0]
    
    set eco [eco::getRlEco]
    $eco addSparePattern $pattern
}
 
sta::define_cmd_args "eco_report" {}
 
proc eco_report { args } {
    set eco [eco::getRlEco]
    $eco reportMetrics
}
 
sta::define_cmd_args "eco_get_num_changes" {}
 
proc eco_get_num_changes { args } {
    set eco [eco::getRlEco]
    return [$eco getNumChanges]
}
 
sta::define_cmd_args "eco_get_num_spare_cells" {}
 
proc eco_get_num_spare_cells { args } {
    set eco [eco::getRlEco]
    return [$eco getNumSpareCells]
}

proc eco_route_changes { args} {
    set eco [eco::getRlEco]
    return [$eco eco_route_changes]
}


#Routing support 
proc route_eco_changes {} {
  set eco [get_eco_pointer]
  $eco routeEcoChanges
}
 
proc write_routed_def {filename} {
  set eco [get_eco_pointer]
  $eco writeRoutedDEF $filename
}


# Add these commands to RlEco.tcl

# Initialize unified environment
sta::define_cmd_args "eco_init_unified" {}

proc eco_init_unified { args } {
    sta::parse_key_args "eco_init_unified" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco initUnifiedEnvironment
}

# Set unified environment mode
sta::define_cmd_args "eco_set_unified_mode" {mode}

proc eco_set_unified_mode { args } {
    sta::parse_key_args "eco_set_unified_mode" args keys {mode} flags {}
    
    set eco [eco::getRlEco]
    $eco setUnifiedMode $keys(mode)
}

# Train unified environment
sta::define_cmd_args "eco_train_unified" {
    [-episodes episodes]
    [-learning_rate learning_rate]
    [-epsilon epsilon]
    [-gamma gamma]
}

proc eco_train_unified { args } {
    sta::parse_key_args "eco_train_unified" args \
        keys {-episodes -learning_rate -epsilon -gamma} \
        flags {}

    set eco [eco::getRlEco]

    # Set defaults
    set episodes [expr {[info exists keys(-episodes)] ? $keys(-episodes) : 1000}]
    set learning_rate [expr {[info exists keys(-learning_rate)] ? $keys(-learning_rate) : 0.001}]
    set epsilon [expr {[info exists keys(-epsilon)] ? $keys(-epsilon) : 0.1}]
    set gamma [expr {[info exists keys(-gamma)] ? $keys(-gamma) : 0.99}]

    $eco trainUnified $episodes $learning_rate $epsilon $gamma
}

# Run generation phase
sta::define_cmd_args "eco_run_generation" {}

proc eco_run_generation { args } {
    sta::parse_key_args "eco_run_generation" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco runUnifiedGeneration
}

# Run implementation phase
sta::define_cmd_args "eco_run_implementation" {}

proc eco_run_implementation { args } {
    sta::parse_key_args "eco_run_implementation" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco runUnifiedImplementation
}

# Run complete unified flow
sta::define_cmd_args "eco_run_unified" {}

proc eco_run_unified { args } {
    sta::parse_key_args "eco_run_unified" args keys {} flags {}
    
    set eco [eco::getRlEco]
    $eco runUnifiedComplete
}

# Get current phase
sta::define_cmd_args "eco_get_phase" {}

proc eco_get_phase { args } {
    sta::parse_key_args "eco_get_phase" args keys {} flags {}
    
    set eco [eco::getRlEco]
    return [$eco getUnifiedPhase]
}

# Get unified state
sta::define_cmd_args "eco_get_unified_state" {}

proc eco_get_unified_state { args } {
    sta::parse_key_args "eco_get_unified_state" args keys {} flags {}
    
    set eco [eco::getRlEco]
    set state [$eco getUnifiedState]
    
    # Convert FloatVector to TCL list
    set tcl_list {}
    for {set i 0} {$i < [FloatVector_size $state]} {incr i} {
        lappend tcl_list [FloatVector_get $state $i]
    }
    
    return $tcl_list
}

# Get unified metrics
sta::define_cmd_args "eco_get_unified_metrics" {}

proc eco_get_unified_metrics { args } {
    sta::parse_key_args "eco_get_unified_metrics" args keys {} flags {}
    
    set eco [eco::getRlEco]
    set metrics [$eco getUnifiedMetrics]
    
    # Format metrics for display
    puts "Unified ECO Metrics:"
    puts "==================="
    dict for {key value} $metrics {
        puts [format "%-30s: %.2f" $key $value]
    }
    
    return $metrics
}

# Example unified workflow command
sta::define_cmd_args "eco_demo_unified" {}

proc eco_demo_unified { args } {
    puts "Running unified ECO flow demo..."
    
    # Initialize
    eco_init
    eco_init_unified
    
    # Load changes
    eco_load_changes "test.eco"
    eco_identify_spare
    
    # Set unified mode
    eco_set_unified_mode "unified"
    
    # Train (small number for demo)
    eco_train_unified -episodes 100 -learning_rate 0.001 -epsilon 0.1 -gamma 0.99
    
    # Run unified flow
    eco_run_unified
    
    # Report results
    eco_get_unified_metrics
}



sta::define_cmd_args "write_eco" {filename}

proc write_eco { args } {
    if { [llength $args] != 1 } {
        utl::error ECO 2001 "write_eco requires exactly one argument: filename"
    }
    
    set filename [lindex $args 0]
    
    set eco [eco::getRlEco]
    if {![$eco writeGeneratedEco $filename]} {
        utl::error ECO 2002 "Failed to write ECO file"
    }
}



