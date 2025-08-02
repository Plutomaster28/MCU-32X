#!/bin/bash

# MCU-32X OpenLane Build Script
# Automated script to run the complete ASIC flow

set -e  # Exit on any error

echo "========================================"
echo "MCU-32X OpenLane Build Script"
echo "========================================"

# Check if we're in the right directory
if [ ! -f "../config.tcl" ]; then
    echo "Error: config.tcl not found. Please run from the MCU-32X root directory."
    exit 1
fi

# Generate a timestamp for the run
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RUN_TAG="mcu32x_${TIMESTAMP}"

echo "Starting OpenLane flow..."
echo "Run tag: ${RUN_TAG}"
echo "Timestamp: ${TIMESTAMP}"

# Copy config to OpenLane designs directory (if needed)
echo "Using config.tcl from parent directory..."

# Run the OpenLane flow
echo "Executing: flow.tcl -design MCU-32X -tag ${RUN_TAG}"

# The actual command (user should run this in OpenLane container)
cat << EOF

PLEASE RUN THE FOLLOWING COMMANDS IN YOUR OPENLANE CONTAINER:

1. Start OpenLane container:
   make mount

2. Run the flow:
   flow.tcl -design /openlane/designs/MCU-32X -tag ${RUN_TAG}

3. Check results in:
   designs/MCU-32X/runs/${RUN_TAG}/results/final/

EOF

echo "========================================"
echo "Script completed. Follow instructions above."
echo "========================================"
