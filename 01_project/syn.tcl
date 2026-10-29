import_device eagle_s20.db -package EG4S20BG256
create_project -device EG4S20BG256 -family EG4 v7f_syn .
read_verilog -dir src -top top
read_adc src/pin_audio_sd.adc
read_sdc src/timing.sdc
optimize_rtl
optimize_gate -packarea
report_area -file syn_area.area
puts "#### SYN_DONE ####"