#!/bin/bash

# Set the Verilog source directory
SRC_DIR="../src"

# Set the output directory for compiled files
OUTPUT_DIR="../build"

# Create the output directory if it doesn't exist
mkdir -p $OUTPUT_DIR

echo "Compiling with Icarus Verilog..."

# Compile the Verilog files (using find to get all .v files)
VERILOG_FILES=$(find $SRC_DIR -name "*.v" | tr '\n' ' ')
echo "Found files: $VERILOG_FILES"

iverilog -o $OUTPUT_DIR/mcu32x.vvp $VERILOG_FILES

# Check if the compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful!"
else
    echo "Compilation failed!"
    exit 1
fi

echo "Running simulation with Icarus Verilog..."
# Run the simulation
vvp $OUTPUT_DIR/mcu32x.vvp

# Check if the simulation was successful
if [ $? -eq 0 ]; then
    echo "Simulation completed successfully!"
else
    echo "Simulation failed!"
    exit 1
fi