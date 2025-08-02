# OpenLane Configuration for MCU-32X
# 32-bit RISC-V style processor with caches, DMA, and FPU

set ::env(DESIGN_NAME) "MCU32X"

# Verilog source files - include all modules
set ::env(VERILOG_FILES) [glob \
    $::env(DESIGN_DIR)/src/top.v \
    $::env(DESIGN_DIR)/src/cpu/*.v \
    $::env(DESIGN_DIR)/src/cpu/pipeline/*.v \
    $::env(DESIGN_DIR)/src/cache/*.v \
    $::env(DESIGN_DIR)/src/memory/*.v \
    $::env(DESIGN_DIR)/src/bus/*.v \
    $::env(DESIGN_DIR)/src/dma/*.v \
    $::env(DESIGN_DIR)/src/interrupt/*.v \
]

# Clock configuration - targeting 200MHz (5ns period)
set ::env(CLOCK_PORT) "clk"
set ::env(CLOCK_PERIOD) "5.0"  ;# 200 MHz - aggressive but achievable in 130nm

# Reset configuration
set ::env(RESET_PORT) "reset"

# Core utilization and density - conservative for first tapeout
set ::env(FP_CORE_UTIL) 35      ;# 35% utilization - plenty of room for routing
set ::env(PL_TARGET_DENSITY) 0.40  ;# Relaxed placement density

# Die size constraints (if needed)
# set ::env(DIE_AREA) "0 0 2000 2000"  ;# 2mm x 2mm die
# set ::env(FP_SIZING) "absolute"

# Pin configuration - let OpenLane auto-assign for now
set ::env(FP_PIN_ORDER_CFG) "$::env(DESIGN_DIR)/pin_order.cfg"

# Routing configuration - conservative settings
set ::env(RT_MAX_LAYER) 6       ;# Use up to metal6 for routing
set ::env(ROUTING_CORES) 4      ;# Parallel routing

# Power configuration
set ::env(VDD_NETS) "vccd1"     ;# Digital VDD
set ::env(GND_NETS) "vssd1"     ;# Digital GND

# Synthesis configuration
set ::env(SYNTH_STRATEGY) "AREA 0"  ;# Balance area and timing
set ::env(SYNTH_BUFFERING) 1
set ::env(SYNTH_SIZING) 1

# Placement configuration
set ::env(PL_RESIZER_DESIGN_OPTIMIZATIONS) 1
set ::env(PL_RESIZER_TIMING_OPTIMIZATIONS) 1

# Timing configuration
set ::env(STA_WRITE_LIB) 1      ;# Write timing libraries
set ::env(RUN_KLAYOUT_XOR) 0    ;# Skip XOR check for now (can be slow)

# DRC/LVS configuration
set ::env(MAGIC_DRC_USE_GDS) 0  ;# Use faster DRC checking
set ::env(RUN_MAGIC_DRC) 1      ;# Run DRC checks
set ::env(RUN_KLAYOUT_DRC) 0    ;# Skip KLayout DRC for now

# Output configuration
set ::env(PRIMARY_SIGNOFF_TOOL) "magic"  ;# Use Magic for final signoff

# Extra settings for complex designs
set ::env(SYNTH_READ_BLACKBOX_LIB) 1     ;# Handle any blackbox modules
set ::env(QUIT_ON_TIMING_VIOLATIONS) 0   ;# Don't quit on timing violations (first pass)
set ::env(QUIT_ON_MAGIC_DRC) 0          ;# Don't quit on DRC violations (first pass)

# Memory configuration (if you have memory macros)
# set ::env(EXTRA_LEFS) "$::env(PDK_ROOT)/$::env(PDK)/libs.ref/sky130_sram_macros/lef/sky130_sram_2kbyte_1rw1r_32x512_8.lef"
# set ::env(EXTRA_GDS_FILES) "$::env(PDK_ROOT)/$::env(PDK)/libs.ref/sky130_sram_macros/gds/sky130_sram_2kbyte_1rw1r_32x512_8.gds"

# Debugging (enable for detailed logs)
set ::env(SYNTH_NO_FLAT) 0      ;# Allow flattening for better optimization