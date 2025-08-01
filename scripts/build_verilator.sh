#!/bin/bash

# Set directories
SRC_DIR="../src"
TEST_DIR="../test"
BUILD_DIR="../build"

# Create build directory
mkdir -p $BUILD_DIR

echo "Building with Verilator..."

# Verilator command with explicit file listing
# Note: Order matters - dependencies should come first
verilator --cc --exe --build \
    -Wno-UNUSED -Wno-UNDRIVEN \
    --top-module tb_mcu32x \
    -o tb_mcu32x \
    --Mdir $BUILD_DIR/obj_dir \
    $TEST_DIR/tb_mcu32x.v \
    $SRC_DIR/cpu/alu.v \
    $SRC_DIR/cpu/register_file.v \
    $SRC_DIR/cpu/control_unit.v \
    $SRC_DIR/cpu/fpu.v \
    $SRC_DIR/cpu/datapath.v \
    $SRC_DIR/cpu/pipeline/fetch.v \
    $SRC_DIR/cpu/pipeline/decode.v \
    $SRC_DIR/cpu/pipeline/execute.v \
    $SRC_DIR/cpu/pipeline/memory.v \
    $SRC_DIR/cpu/pipeline/writeback.v \
    $SRC_DIR/cache/icache.v \
    $SRC_DIR/cache/dcache.v \
    $SRC_DIR/bus/internal_bus.v \
    $SRC_DIR/bus/system_bus.v \
    $SRC_DIR/memory/rom.v \
    $SRC_DIR/memory/flash.v \
    $SRC_DIR/memory/ddr_controller.v \
    $SRC_DIR/dma/dma_controller.v \
    $SRC_DIR/interrupt/interrupt_controller.v \
    $SRC_DIR/top.v

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Verilator compilation successful!"
    echo "Running simulation..."
    
    # Change to build directory and run simulation
    cd $BUILD_DIR/obj_dir
    ./tb_mcu32x
    
    if [ $? -eq 0 ]; then
        echo "Simulation completed successfully!"
        echo "VCD file should be in obj_dir/mcu32x.vcd"
        echo "To view waveforms: gtkwave mcu32x.vcd"
    else
        echo "Simulation failed!"
        exit 1
    fi
else
    echo "Verilator compilation failed!"
    exit 1
fi
