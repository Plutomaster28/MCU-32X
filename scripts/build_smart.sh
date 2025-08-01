#!/bin/bash

# Auto-detecting build script for different environments
echo "MCU-32X Build Script - Auto-detecting environment..."

# Detect environment
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    echo "Detected Windows environment (MSYS2/UCRT64)"
    ENV="windows"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if grep -qi microsoft /proc/version 2>/dev/null; then
        echo "Detected WSL (Windows Subsystem for Linux)"
        ENV="wsl"
    else
        echo "Detected native Linux"
        ENV="linux"
    fi
else
    echo "Detected other Unix-like environment"
    ENV="unix"
fi

echo "Environment: $ENV"
echo ""

# Choose build method based on environment and available tools
if command -v verilator >/dev/null 2>&1; then
    echo "Verilator found"
    if [[ "$ENV" == "windows" ]]; then
        echo "Using Windows-specific Verilator build..."
        ./build_windows.sh
    else
        echo "Using standard Verilator build..."
        ./build_auto.sh
    fi
elif command -v iverilog >/dev/null 2>&1; then
    echo "Icarus Verilog found, using iverilog..."
    ./build_icarus.sh
else
    echo "ERROR: No Verilog simulator found!"
    echo ""
    echo "Please install one of the following:"
    echo "  - Verilator: Fast, good for synthesis-ready code"
    echo "  - Icarus Verilog: Simple, good for learning"
    echo ""
    echo "Installation instructions:"
    echo "  Ubuntu/WSL: sudo apt install verilator iverilog gtkwave"
    echo "  UCRT64: pacman -S mingw-w64-ucrt-x86_64-verilator mingw-w64-ucrt-x86_64-iverilog"
    exit 1
fi
