#!/bin/bash

# Set directories
SRC_DIR="../src"
TEST_DIR="../test"
BUILD_DIR="../build"

# Create build directory
mkdir -p $BUILD_DIR

echo "Auto-discovering Verilog files..."

# Find all .v files in src directory
VERILOG_FILES=$(find $SRC_DIR -name "*.v" | tr '\n' ' ')
TEST_FILES=$(find $TEST_DIR -name "*.v" | tr '\n' ' ')

echo "Found source files:"
echo $VERILOG_FILES
echo "Found test files:"
echo $TEST_FILES

echo "Building with Verilator..."

# Verilator command with auto-discovered files
verilator --cc --exe --build \
    -Wno-UNUSED -Wno-UNDRIVEN -Wno-UNOPTFLAT -Wno-WIDTHTRUNC \
    --no-timing \
    --top-module tb_mcu32x \
    -o tb_mcu32x \
    --Mdir $BUILD_DIR/obj_dir \
    $TEST_FILES \
    $VERILOG_FILES

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
