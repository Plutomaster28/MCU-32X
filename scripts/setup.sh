#!/bin/bash

# Make all scripts executable
chmod +x build.sh
chmod +x build_verilator.sh
chmod +x build_auto.sh

echo "Made all scripts executable!"
echo ""
echo "Available build scripts:"
echo "  ./build.sh           - Uses Icarus Verilog (iverilog)"
echo "  ./build_verilator.sh - Uses Verilator (explicit file list)"
echo "  ./build_auto.sh      - Uses Verilator (auto-discovery)"
echo ""
echo "For Verilator simulation in WSL, try:"
echo "  cd test"
echo "  ../scripts/build_auto.sh"
