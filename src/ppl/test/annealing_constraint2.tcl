# gcd_nangate45 IO placement
source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_def gcd.def

set_io_pin_constraint -direction INPUT -region bottom:*
set_io_pin_constraint -direction OUTPUT -region top:*
place_pins -hor_layers metal3 -ver_layers metal2 -corner_avoidance 0 -min_distance 0.12 -annealing

set def_file [make_result_file annealing_constraint2.def]

write_def $def_file

diff_file annealing_constraint2.defok $def_file
