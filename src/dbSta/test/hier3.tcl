source "helpers.tcl"
read_lef example1.lef
read_liberty example1_typ.lib
read_verilog op_const.v
link_design top_wrap -hier
set v_file [make_result_file hier3_out.v]
write_verilog $v_file
diff_files $v_file hier3_out.vok
