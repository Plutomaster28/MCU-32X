#!/bin/bash

# Windows/UCRT64 specific build script for Verilator
# Set directories
SRC_DIR="../src"
TEST_DIR="../test"
BUILD_DIR="../build"

# Create build directory
mkdir -p $BUILD_DIR

echo "Auto-discovering Verilog files (Windows/UCRT64)..."

# Find all .v files in src directory
VERILOG_FILES=$(find $SRC_DIR -name "*.v" | tr '\n' ' ')
TEST_FILES=$(find $TEST_DIR -name "*.v" | tr '\n' ' ')

echo "Found source files:"
echo $VERILOG_FILES
echo "Found test files:"
echo $TEST_FILES

echo "Building with Verilator on Windows..."

# Verilator command for Windows/UCRT64
verilator --cc --exe --build \
    -Wno-UNUSED -Wno-UNDRIVEN -Wno-UNOPTFLAT -Wno-WIDTHTRUNC \
    -Wno-CASEINCOMPLETE -Wno-INFINITELOOP \
    --timing \
    --trace \
    --top-module tb_mcu32x \
    -o tb_mcu32x.exe \
    --Mdir $BUILD_DIR/obj_dir \
    $TEST_FILES \
    $TEST_DIR/tb_mcu32x_main_windows.cpp \
    $VERILOG_FILES

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Verilator compilation successful!"
    echo "Running simulation..."
    
    # Change to build directory and run simulation
    cd $BUILD_DIR/obj_dir
    ./tb_mcu32x.exe
    
    if [ $? -eq 0 ]; then
        echo "Simulation completed successfully!"
        echo "VCD file should be in obj_dir/mcu32x.vcd"
        echo "To view waveforms: gtkwave mcu32x.vcd"
        echo ""
        echo "Note: On Windows, you may need to install GTKWave separately"
        echo "Download from: https://sourceforge.net/projects/gtkwave/"
    else
        echo "Simulation failed!"
        exit 1
    fi
else
    echo "Verilator compilation failed!"
    echo ""
    echo "If you see linking errors, try:"
    echo "1. Make sure you have the full UCRT64 development environment"
    echo "2. Try using WSL instead for a more compatible Linux environment"
    echo "3. Or use the build.sh script with iverilog instead"
    exit 1
fi
