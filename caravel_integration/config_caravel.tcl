# Caravel OpenLane Configuration for MCU-32X Integration
# This config builds the MCU-32X integrated with Caravel harness

set ::env(DESIGN_NAME) user_project_wrapper

# Source files - include both MCU-32X and wrapper
set ::env(VERILOG_FILES) [glob \
    $::env(DESIGN_DIR)/caravel_integration/user_project_wrapper.v \
    $::env(DESIGN_DIR)/src/top.v \
    $::env(DESIGN_DIR)/src/cpu/*.v \
    $::env(DESIGN_DIR)/src/cpu/pipeline/*.v \
    $::env(DESIGN_DIR)/src/cache/*.v \
    $::env(DESIGN_DIR)/src/memory/*.v \
    $::env(DESIGN_DIR)/src/bus/*.v \
    $::env(DESIGN_DIR)/src/dma/*.v \
    $::env(DESIGN_DIR)/src/interrupt/*.v \
]

# Include Caravel macros and defines
set ::env(VERILOG_INCLUDE_DIRS) [glob $::env(CARAVEL_ROOT)/verilog/rtl/defines]

# Clock configuration - use Caravel's clock
set ::env(CLOCK_PORT) "wb_clk_i"
set ::env(CLOCK_PERIOD) "25.0"  ;# 40MHz - Caravel's typical clock

# Reset configuration
set ::env(RESET_PORT) "wb_rst_i"

# Caravel user project area constraints
set ::env(DIE_AREA) "0 0 2920 3520"  ;# Caravel user area size
set ::env(FP_SIZING) "absolute"

# Core utilization - higher for Caravel integration
set ::env(FP_CORE_UTIL) 20
set ::env(PL_TARGET_DENSITY) 0.4

# Pin configuration for Caravel
set ::env(FP_IO_MODE) 0  ;# Use Caravel's IO pattern

# Power configuration for Caravel
set ::env(VDD_NETS) "vccd1"
set ::env(GND_NETS) "vssd1"

# PDN configuration for user area
set ::env(FP_PDN_VPITCH) 180
set ::env(FP_PDN_HPITCH) 180
set ::env(FP_PDN_VWIDTH) 5.0
set ::env(FP_PDN_HWIDTH) 5.0

# Routing layers (same as before)
set ::env(RT_MAX_LAYER) "met5"
set ::env(RT_MIN_LAYER) "met1"
set ::env(GLB_RT_MINLAYER) 1
set ::env(GLB_RT_MAXLAYER) 5

# Synthesis configuration
set ::env(SYNTH_STRATEGY) "AREA 0"
set ::env(SYNTH_BUFFERING) 1
set ::env(SYNTH_SIZING) 1

# Don't quit on violations for first integration
set ::env(QUIT_ON_TIMING_VIOLATIONS) 0
set ::env(QUIT_ON_MAGIC_DRC) 0
set ::env(QUIT_ON_SYNTH_CHECKS) 0

# Caravel-specific settings
set ::env(FP_WELLTAP_CELL) "sky130_fd_sc_hd__tapvpwrvgnd_1"
set ::env(FP_ENDCAP_CELL) "sky130_fd_sc_hd__decap_3"
