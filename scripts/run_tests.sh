#!/bin/bash

# This script runs the testbenches to verify the functionality of the design.

# Compile the Verilog files
iverilog -o mcu32x_tb testbench/cpu_tb.v testbench/cache_tb.v testbench/memory_tb.v src/*.v src/cpu/pipeline/*.v src/cache/*.v src/bus/*.v src/memory/*.v src/dma/*.v src/interrupt/*.v

# Run the testbenches
vvp mcu32x_tb

# Check the exit status of the test run
if [ $? -eq 0 ]; then
    echo "All tests passed successfully."
else
    echo "Some tests failed. Please check the output for details."
fi