#!/bin/bash

# Set the Verilog source directory
SRC_DIR="../src"

# Set the output directory for compiled files
OUTPUT_DIR="../build"

# Create the output directory if it doesn't exist
mkdir -p $OUTPUT_DIR

# Compile the Verilog files
iverilog -o $OUTPUT_DIR/mcu32x.vvp $SRC_DIR/cpu/*.v $SRC_DIR/cpu/pipeline/*.v $SRC_DIR/cache/*.v $SRC_DIR/bus/*.v $SRC_DIR/memory/*.v $SRC_DIR/dma/*.v $SRC_DIR/interrupt/*.v $SRC_DIR/top.v

# Check if the compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful!"
else
    echo "Compilation failed!"
    exit 1
fi

# Run the simulation
vvp $OUTPUT_DIR/mcu32x.vvp

# Check if the simulation was successful
if [ $? -eq 0 ]; then
    echo "Simulation completed successfully!"
else
    echo "Simulation failed!"
    exit 1
fi