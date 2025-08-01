#!/bin/bash

# Make all scripts executable
chmod +x build.sh
chmod +x build_verilator.sh
chmod +x build_auto.sh
chmod +x build_windows.sh
chmod +x build_icarus.sh
chmod +x build_smart.sh

echo "Made all scripts executable!"
echo ""
echo "Available build scripts:"
echo "  ./build_smart.sh     - Auto-detects environment and chooses best method (RECOMMENDED)"
echo "  ./build.sh           - Uses Icarus Verilog (iverilog) - original script"
echo "  ./build_auto.sh      - Uses Verilator (Linux/WSL)"
echo "  ./build_windows.sh   - Uses Verilator (Windows/UCRT64)"
echo "  ./build_icarus.sh    - Uses Icarus Verilog (cross-platform)"
echo ""
echo "For your environment, I recommend:"
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    echo "  ./build_smart.sh  (will auto-choose between Verilator and iverilog)"
    echo "  or ./build_icarus.sh  (more reliable on Windows)"
else
    echo "  ./build_smart.sh  (will auto-choose the best simulator)"
fi
