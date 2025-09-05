# OpenLane Configuration for MCU-32X
# 32-bit RISC-V RV32I processor with expanded I/O for demonstration
# Targeting 180nm process node compatibility

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

# Clock configuration - more conservative for 180nm compatibility
set ::env(CLOCK_PORT) "clk"
set ::env(CLOCK_PERIOD) "10.0"  ;# 100 MHz - conservative for 180nm and demo purposes

# Reset configuration
set ::env(RESET_PORT) "reset"

# Core utilization and density - very low for demonstration and easy routing
set ::env(FP_CORE_UTIL) 5       ;# Very low utilization for demo - easier routing
set ::env(PL_TARGET_DENSITY) 0.15   ;# Very relaxed placement density

# Die size constraints - much larger for demonstration with many I/Os
set ::env(DIE_AREA) "0 0 2000 2000"     ;# 2mm x 2mm die - large for demos
set ::env(FP_SIZING) "absolute"         ;# Use absolute die size
set ::env(CORE_AREA) "100 100 1900 1900" ;# Large core area for plenty of space
set ::env(BOTTOM_MARGIN_MULT) 4         ;# Larger margins for I/O pins
set ::env(TOP_MARGIN_MULT) 4
set ::env(LEFT_MARGIN_MULT) 4
set ::env(RIGHT_MARGIN_MULT) 4

# Power grid settings - larger pitch for demo chip
set ::env(FP_PDN_VPITCH) 100.0      ;# Larger vertical power grid pitch
set ::env(FP_PDN_HPITCH) 100.0      ;# Larger horizontal power grid pitch
set ::env(FP_PDN_VWIDTH) 5.0        ;# Wider vertical power rails  
set ::env(FP_PDN_HWIDTH) 5.0        ;# Wider horizontal power rails

# Pin configuration - let OpenLane auto-assign for now
# set ::env(FP_PIN_ORDER_CFG) "$::env(DESIGN_DIR)/pin_order.cfg"

# Routing configuration - fix layer issues
set ::env(RT_MAX_LAYER) "met5"  ;# Use up to metal5 for routing (safer)
set ::env(RT_MIN_LAYER) "met1"  ;# Minimum routing layer
set ::env(ROUTING_CORES) 4      ;# Parallel routing
set ::env(GLB_RT_MINLAYER) 1    ;# Global routing min layer (met1)
set ::env(GLB_RT_MAXLAYER) 5    ;# Global routing max layer (met5)

# Clock routing constraints
set ::env(CTS_CLK_MIN_LAYER) 3  ;# Clock minimum layer (met3)
set ::env(CTS_CLK_MAX_LAYER) 5  ;# Clock maximum layer (met5)

# IO-specific configurations for high pin count designs (demonstration)
set ::env(FP_IO_MODE) 1         ;# Random IO placement mode
set ::env(FP_IO_MIN_DISTANCE) 10 ;# Larger minimum distance between IO pins for demo
set ::env(FP_IO_HLENGTH) 50     ;# Longer horizontal I/O pins
set ::env(FP_IO_VLENGTH) 50     ;# Longer vertical I/O pins

# Power configuration
set ::env(VDD_NETS) "vccd1"     ;# Digital VDD
set ::env(GND_NETS) "vssd1"     ;# Digital GND

# Additional power grid settings for demo chip
set ::env(FP_PDN_CORE_RING) 1          ;# Enable core ring
set ::env(FP_PDN_CORE_RING_VWIDTH) 5.0 ;# Wider core ring vertical width
set ::env(FP_PDN_CORE_RING_HWIDTH) 5.0 ;# Wider core ring horizontal width
set ::env(FP_PDN_CORE_RING_VSPACING) 20.0 ;# Core ring spacing
set ::env(FP_PDN_CORE_RING_HSPACING) 20.0 ;# Core ring spacing

# Synthesis configuration
set ::env(SYNTH_STRATEGY) "AREA 0"  ;# Balance area and timing
set ::env(SYNTH_BUFFERING) 1
set ::env(SYNTH_SIZING) 1

# Placement configuration
set ::env(PL_RESIZER_DESIGN_OPTIMIZATIONS) 1
set ::env(PL_RESIZER_TIMING_OPTIMIZATIONS) 1

# Layer configuration - prevent substrate layer usage
set ::env(GRT_ALLOW_CONGESTION) 1       ;# Allow some congestion to avoid bad layers

# Timing configuration
set ::env(STA_WRITE_LIB) 1      ;# Write timing libraries
set ::env(RUN_KLAYOUT_XOR) 0    ;# Skip XOR check for now (can be slow)

# DRC/LVS configuration
set ::env(MAGIC_DRC_USE_GDS) 0  ;# Use faster DRC checking
set ::env(RUN_MAGIC_DRC) 1      ;# Run DRC checks
set ::env(RUN_KLAYOUT_DRC) 0    ;# Skip KLayout DRC for now

# Output configuration
set ::env(PRIMARY_SIGNOFF_TOOL) "magic"  ;# Use Magic for final signoff

# Extra settings for demonstration design with many I/Os
set ::env(SYNTH_READ_BLACKBOX_LIB) 1     ;# Handle any blackbox modules
set ::env(QUIT_ON_TIMING_VIOLATIONS) 0   ;# Don't quit on timing violations (demo mode)
set ::env(QUIT_ON_MAGIC_DRC) 0          ;# Don't quit on DRC violations (demo mode)
set ::env(QUIT_ON_SYNTH_CHECKS) 0       ;# Don't quit on synthesis check warnings
set ::env(FP_IO_UNMATCHED_ERROR) 0      ;# Don't error on unmatched I/O pins

# 180nm process compatibility settings
set ::env(SYNTH_MAX_FANOUT) 6           ;# Lower fanout for older process
set ::env(PL_RESIZER_MAX_SLEW_MARGIN) 15 ;# More conservative slew margins
set ::env(PL_RESIZER_MAX_CAP_MARGIN) 15  ;# More conservative capacitance margins

# Memory configuration (if you have memory macros)
# set ::env(EXTRA_LEFS) "$::env(PDK_ROOT)/$::env(PDK)/libs.ref/sky130_sram_macros/lef/sky130_sram_2kbyte_1rw1r_32x512_8.lef"
# set ::env(EXTRA_GDS_FILES) "$::env(PDK_ROOT)/$::env(PDK)/libs.ref/sky130_sram_macros/gds/sky130_sram_2kbyte_1rw1r_32x512_8.gds"

# Debugging (enable for detailed logs)
set ::env(SYNTH_NO_FLAT) 0      ;# Allow flattening for better optimization