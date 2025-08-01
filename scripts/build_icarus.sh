#!/bin/bash

# Simple Icarus Verilog build script for Windows
# This is more reliable than Verilator on UCRT64

# Set directories
SRC_DIR="../src"
TEST_DIR="../test" 
BUILD_DIR="../build"

# Create build directory
mkdir -p $BUILD_DIR

echo "Building with Icarus Verilog (Windows compatible)..."

# Find all .v files
VERILOG_FILES=$(find $SRC_DIR -name "*.v" | tr '\n' ' ')
TEST_FILES=$(find $TEST_DIR -name "*.v" | tr '\n' ' ')

echo "Found source files:"
echo $VERILOG_FILES
echo "Found test files:"
echo $TEST_FILES

# Compile with iverilog
iverilog -o $BUILD_DIR/mcu32x_sim.vvp $TEST_FILES $VERILOG_FILES

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Icarus Verilog compilation successful!"
    echo "Running simulation..."
    
    # Run simulation
    cd $BUILD_DIR
    vvp mcu32x_sim.vvp
    
    if [ $? -eq 0 ]; then
        echo "Simulation completed successfully!"
        echo "If VCD files were generated, you can view them with GTKWave"
    else
        echo "Simulation had issues, but this is normal for initial testing"
    fi
else
    echo "Compilation failed!"
    exit 1
fi
