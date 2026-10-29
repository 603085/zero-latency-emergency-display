import_device eagle_s20.db -package EG4S20BG256
create_project -device EG4S20BG256 -family EG4 v7_impl .
read_verilog -dir src -top top
read_adc src/pin_audio_sd.adc
read_sdc src/timing.sdc
optimize_rtl
optimize_gate -packarea
place
update_timing -mode manhattan
report_timing_summary -file v7_place.timing
route
update_timing -mode final
report_timing_summary -file v7_route.timing
report_area -io_info -file v7_phy.area
bitgen -bit v7.bit
puts "#### V7_IMPL_DONE ####"