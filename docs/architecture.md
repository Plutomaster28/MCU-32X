# MCU-32X Processor Architecture

## Overview
The MCU-32X is a 32-bit RISC processor designed with a focus on simplicity and efficiency. It is based on a custom ISA inspired by RISC-V RV32IM and MIPS architectures. The processor is suitable for running homebrew operating systems and game kernels, providing performance comparable to late 90s RISC workstations or a stripped-down Pentium II.

## Architecture Specifications

### CPU Type
- **Architecture**: 32-bit RISC
- **ISA**: Custom MIPS-like or RISC-V RV32IM

### Pipeline
- **Stages**: 5 to 7 stages, in-order execution
- **Stages Include**:
  - Fetch
  - Decode
  - Execute
  - Memory Access
  - Writeback

### Registers
- **General Purpose Registers**: 32 × 32-bit
- **Special Registers**: Additional registers for control and status

### Floating Point Unit (FPU)
- **Type**: Optional
- **Functionality**: Can handle fixed-point operations for a retro computing experience

### Clock Speed
- **Frequency**: Approximately 75 to 125 MHz
- **Technology**: Designed for 130 nm process technology

### Cache
- **Instruction Cache (I-cache)**: 8 to 16 KB
- **Data Cache (D-cache)**: 8 to 16 KB
- **Cache Type**: Write-back support

### Bus Architecture
- **Internal Bus**: 64-bit
- **System Bus**: 32-bit interface to main RAM

### Memory Interface
- **External Memory**: DDR1/DDR2 interface
- **Boot Memory**: Optional ROM/Flash for booting

### Special Features
- **DMA Controller**: Facilitates data transfer to GPU VRAM and sound RAM
- **Hardware Multiply/Divide**: Supports efficient arithmetic operations
- **Interrupt Controller**: Manages interrupts for responsive processing

## Expected Performance
The MCU-32X aims to deliver performance on par with late 90s RISC workstations, making it an ideal platform for educational purposes, hobbyist projects, and retro computing enthusiasts. Its architecture is designed to be straightforward, allowing for easy modifications and enhancements.