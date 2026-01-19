read_lef NangateOpenCellLibrary.tech.lef
read_lef NangateOpenCellLibrary.macro.mod.lef
read_def 6_final.def
set db [ord::get_db]
set block [ord::get_db_block]
set die_area [$block getDieArea]
puts 
# Load design as above
read_lef tech.lef
read_def design.def

# Get the database and block (top-level design)
set db [ord::get_db]
set block [ord::get_db_block]


puts "Dumping physical data for analysis"

set fp [open "grid_dump.txt" w]

# Dump Die Area (overall chip boundaries in DB units, e.g., microns * dbu_per_micron)
set die_area [$block getDieArea]
puts "Die Area: xMin=[$die_area xMin] yMin=[$die_area yMin] xMax=[$die_area xMax] yMax=[$die_area yMax]"

# Dump Core Area (placement area)
set core_area [$block getCoreArea]
puts $fp "Core Area: xMin=[$core_area xMin] yMin=[$core_area yMin] xMax=[$core_area xMax] yMax=[$core_area yMax]"


# Dump Routing Track Grids (corrected: use getTrackGrids instead of getTracks)
set tech [$db getTech]
set track_grids [$block getTrackGrids]  ;# This is the key method – gets all track grids
 

if {[llength $track_grids] == 0} {
    puts "Warning: No track grids found in the design. Check your tech LEF for TRACKS definitions."
} else {
    puts  $fp "Number of Track Grids: [llength $track_grids]"
    foreach track_grid $track_grids {
        set layer [$track_grid getTechLayer]
        if {[$layer getRoutingLevel] > 0} {  ;# Only routing layers
            set layer_name [$layer getName]
            set direction [$layer getDirection]
            set track_width [$layer getWidth]
 
            # Get position lists
            set x_list [$track_grid getGridX]
            set y_list [$track_grid getGridY]
 
            # Primary: Compute num_tracks from list length (safe and aggregate)
            if {$direction == "VERTICAL"} {
                set track_pos_list $x_list
                set span_list $y_list
            } elseif {$direction == "HORIZONTAL"} {
                set track_pos_list $y_list
                set span_list $x_list
            } else {
                set track_pos_list {}
                set span_list {}
            }
            set num_tracks [llength $track_pos_list]
 
            # Fallback/Alternative: Compute from patterns if list is empty or for verification
            if {$num_tracks == 0} {
                set num_tracks 0
                # Sum from X patterns
                set num_x_patterns [$track_grid getNumGridPatternsX]
                for {set i 0} {$i < $num_x_patterns} {incr i} {
                    set pattern [$track_grid getGridPatternX $i]
                    incr num_tracks [$pattern getCount]  ;# getCount should be valid on pattern objects
                }
                # Sum from Y patterns
                set num_y_patterns [$track_grid getNumGridPatternsY]
                for {set i 0} {$i < $num_y_patterns} {incr i} {
                    set pattern [$track_grid getGridPatternY $i]
                    incr num_tracks [$pattern getCount]
                }
            }
 
            if {$num_tracks == 0} {
                puts "  Warning: No tracks found for layer $layer_name (empty grid/patterns)."
                continue
            }
 
            # Compute offset, min/max, avg pitch from position list
            set track_offset [lindex $track_pos_list 0]
            set grid_min [lindex $track_pos_list 0]
            set grid_max [lindex $track_pos_list end]
            set span_min [lindex $span_list 0]
            set span_max [lindex $span_list end]
            set total_spacing 0.0
            for {set i 1} {$i < $num_tracks} {incr i} {
                set prev [lindex $track_pos_list [expr {$i - 1}]]
                set curr [lindex $track_pos_list $i]
                set total_spacing [expr {$total_spacing + ($curr - $prev)}]
            }
            set track_pitch [expr {$num_tracks > 1 ? $total_spacing / ($num_tracks - 1) : 0}]
 
            # Optional: Average spacing fallback if needed
            # set avg_spacing [$track_grid getAverageTrackSpacing]
 
            puts $fp "Layer: $layer_name Direction=$direction NumTracks=$num_tracks Width=$track_width Offset=$track_offset AvgPitch=$track_pitch"
            puts $fp "  Grid Bounds: Min=$grid_min Max=$grid_max SpanMin=$span_min SpanMax=$span_max"
        }
    }
}

# Step 2: Dump All Placed Cells
    puts $fp "\n# --- All Placed Cells ---"
    set all_insts [$block getInsts]  ;# Get all instances
    puts $fp "TotalCells=[llength $all_insts]"
    foreach inst $all_insts {
        set inst_name [$inst getName]
        set master [$inst getMaster]
        set master_name [$master getName]
        set origin [$inst getOrigin]  ;# list {x y}
        set x [lindex $origin 0]
        set y [lindex $origin 1]
        set bbox [$inst getBBox]  ;# dbBox object
        set llx [$bbox xMin]
        set lly [$bbox yMin]
        set urx [$bbox xMax]
        set ury [$bbox yMax]
        set orient [$inst getOrient]  ;# e.g., N, S, E, W, FN, FS, etc.
        set site [$master getSite]    ;# Site name for placement grid
        puts $fp "Cell=$inst_name Type=$master_name OriginX=$x OriginY=$y BBox=$llx,$lly,$urx,$ury Orient=$orient Site=$site"
    }
 
    # Step 3: Dump Spare Cells (filtered from above)
    puts $fp "\n# --- Spare Cells ---"
    set spare_list {}
    foreach inst $all_insts {
        set inst_name [$inst getName]
        set master_name [[$inst getMaster] getName]
        if {[is_spare $inst_name $master_name]} {
            lappend spare_list $inst  ;# Collect for dumping
        }
    }
    puts $fp "TotalSpares=[llength $spare_list]"
    foreach inst $spare_list {
        # Same details as above
        set inst_name [$inst getName]
        set master_name [[$inst getMaster] getName]
        set origin [$inst getOrigin]
        set x [lindex $origin 0]
        set y [lindex $origin 1]
        set bbox [$inst getBBox]
        set llx [$bbox xMin]
        set lly [$bbox yMin]
        set urx [$bbox xMax]
        set ury [$bbox yMax]
        set orient [$inst getOrient]
        set site [[$inst getMaster] getSite]
        puts $fp "Spare=$inst_name Type=$master_name OriginX=$x OriginY=$y BBox=$llx,$lly,$urx,$ury Orient=$orient Site=$site"
    }
 
    close $fp
    puts "Dump complete: $fp"
}
 




close $fp


# NangateOpenCellLibrary.macro.lef       fakeram45_128x32.lef   fakeram45_256x96.lef  fakeram45_64x32.lef
# NangateOpenCellLibrary.macro.mod.lef   fakeram45_2048x39.lef  fakeram45_32x32.lef   fakeram45_64x62.lef
# NangateOpenCellLibrary.macro.rect.lef  fakeram45_256x16.lef   fakeram45_32x64.lef   fakeram45_64x64.lef
# NangateOpenCellLibrary.tech.lef        fakeram45_256x32.lef   fakeram45_512x64.lef  fakeram45_64x7.lef
# fakeram45_1024x32.lef                  fakeram45_256x34.lef   fakeram45_64x124.lef  fakeram45_64x96.lef
# fakeram45_128x116.lef                  fakeram45_256x48.lef   fakeram45_64x15.lef
# fakeram45_128x256.lef                  fakeram45_256x95.lef   fakeram45_64x21.lef
