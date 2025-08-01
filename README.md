# MCU-32X Processor

## Overview
The MCU-32X is a 32-bit RISC processor designed for educational and experimental purposes. It features a custom ISA inspired by RISC-V RV32IM and MIPS architectures, making it suitable for running homebrew operating systems and game kernels.

## Specifications
- **Architecture**: 32-bit RISC (RISC-V RV32IM or MIPS-like custom ISA)
- **Pipeline**: 5–7 stage, in-order execution
- **Registers**: 32 × 32-bit general purpose registers, plus special registers
- **FPU**: Optional fixed-point unit for retro computing
- **Clock Speed**: 75–125 MHz (realistic on 130 nm technology)
- **Cache**:
  - I-cache: 8–16 KB
  - D-cache: 8–16 KB
  - Write-back support
- **Bus**: 64-bit internal bus, 32-bit system bus to main RAM
- **Memory**:
  - External DDR1/DDR2 interface
  - Optional ROM/Flash boot
- **Special Features**:
  - DMA controller for GPU VRAM/sound RAM transfers
  - Hardware multiply/divide support
  - Interrupt controller

## Expected Performance
The MCU-32X aims to deliver performance comparable to late 90s RISC workstations or a stripped-down Pentium II in integer workloads.

## Project Structure
The project is organized into several directories:
- **src**: Contains the source code for the CPU, cache, bus, memory, DMA, and interrupt controller.
- **testbench**: Contains testbenches for verifying the functionality of the CPU, cache, and memory components.
- **scripts**: Contains scripts for building the project and running tests.
- **docs**: Contains documentation related to the architecture, ISA, and pipeline stages.

## Getting Started
To build the project, run the `build.sh` script located in the `scripts` directory. After building, you can run the testbenches using the `run_tests.sh` script.

## Documentation
For detailed information on the architecture, instruction set, and pipeline stages, refer to the documentation files in the `docs` directory.