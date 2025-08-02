# MCU-32X: 32-bit RISC-V Style Processor

![MCU-32X Block Diagram](docs/block_diagram.png)

## Overview

MCU-32X is a 32-bit RISC-V style processor designed for the SkyWater 130nm process node using 100% open-source EDA tools. This design demonstrates a complete CPU implementation from RTL to GDSII, achieving 200MHz operation in a compact 479µm × 479µm die area.

## Features

- **32-bit RISC-V style architecture**
- **200MHz target clock frequency** (5ns period)
- **Pipelined execution** with 5-stage pipeline (Fetch, Decode, Execute, Memory, Writeback)
- **Arithmetic Logic Unit (ALU)** with standard operations
- **Register file** with 32 registers
- **Floating Point Unit (FPU)** for arithmetic operations
- **Cache subsystem** (ICache + DCache)
- **DMA controller** for high-speed data transfers
- **Interrupt controller** for system management
- **Memory interfaces** (DDR, Flash, ROM)
- **Internal and system bus architecture**

## Specifications

| Parameter | Value |
|-----------|-------|
| Architecture | 32-bit RISC-V style |
| Process Node | SkyWater 130nm |
| Target Frequency | 200MHz |
| Die Size | 479.78µm × 478.72µm |
| Core Area | 480µm × 480µm |
| Core Utilization | 10% (conservative for first tapeout) |
| Power Grid Pitch | 50µm (horizontal and vertical) |
| Metal Layers Used | met1 through met5 |
| I/O Pins | 18 (reduced for initial tapeout) |

## Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                        MCU-32X                             │
├─────────────────┬───────────────────┬─────────────────────┤
│   Fetch Stage   │   Decode Stage    │   Execute Stage     │
│                 │                   │                     │
│ ┌─────────────┐ │ ┌───────────────┐ │ ┌─────────────────┐ │
│ │ Instruction │ │ │ Control Unit  │ │ │      ALU        │ │
│ │   Memory    │ │ │               │ │ │                 │ │
│ │             │ │ │ ┌───────────┐ │ │ │ ┌─────────────┐ │ │
│ │ PC Register │ │ │ │ Decoder   │ │ │ │ │    FPU      │ │ │
│ └─────────────┘ │ │ └───────────┘ │ │ │ └─────────────┘ │ │
└─────────────────┴───────────────────┴─────────────────────┤
├─────────────────┬───────────────────┬─────────────────────┤
│  Memory Stage   │  Writeback Stage  │   Cache Subsystem   │
│                 │                   │                     │
│ ┌─────────────┐ │ ┌───────────────┐ │ ┌─────────────────┐ │
│ │   Memory    │ │ │ Register File │ │ │     ICache      │ │
│ │ Controller  │ │ │               │ │ │                 │ │
│ │             │ │ │    32 x 32    │ │ │ ┌─────────────┐ │ │
│ │ DDR/Flash   │ │ │   Registers   │ │ │ │   DCache    │ │ │
│ └─────────────┘ │ └───────────────┘ │ │ └─────────────┘ │ │
└─────────────────┴───────────────────┴─────────────────────┤
├─────────────────────────────────────────────────────────────┤
│              System Infrastructure                          │
│                                                             │
│ ┌─────────────┐ ┌─────────────┐ ┌─────────────────────────┐ │
│ │     DMA     │ │ Interrupt   │ │       Bus Matrix        │ │
│ │ Controller  │ │ Controller  │ │                         │ │
│ │             │ │             │ │ Internal Bus ↔ System   │ │
│ └─────────────┘ └─────────────┘ └─────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Pin Configuration

| Pin Name | Direction | Width | Description |
|----------|-----------|-------|-------------|
| clk | Input | 1 | System clock (200MHz) |
| reset | Input | 1 | Active high reset |
| result_low[7:0] | Output | 8 | Lower 8 bits of ALU result |
| address_low[7:0] | Output | 8 | Lower 8 bits of memory address |
| mem_read | Output | 1 | Memory read enable |
| mem_write | Output | 1 | Memory write enable |

*Note: Pin count reduced to 18 total for initial tapeout to fit die constraints*

## Getting Started

### Prerequisites

- Docker (for OpenLane)
- WSL2 (if using Windows)
- Git

### Building the Design

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Plutomaster28/MCU-32X.git
   cd MCU-32X
   ```

2. **Start OpenLane Docker container:**
   ```bash
   make mount
   ```

3. **Run the ASIC flow:**
   ```bash
   flow.tcl -design MCU-32X
   ```

4. **Results will be in:**
   ```
   runs/[run_name]/results/final/
   ├── MCU32X.gds      # GDSII layout
   ├── MCU32X.lef      # Abstract view
   ├── MCU32X.nl.v     # Gate-level netlist
   └── MCU32X.def      # Placement & routing
   ```

## Verification Results

### ✅ Design Rule Check (DRC)
- **Status:** 1 minor violation (N-well tap)
- **Impact:** Low risk, acceptable for educational tapeouts

### ✅ Layout vs. Schematic (LVS)
- **Status:** PASSED - No mismatches
- **Total errors:** 0

### ✅ Static Timing Analysis (STA)
- **Status:** PASSED - No violations
- **Target frequency:** 200MHz achieved

## License

This project is open source and available under the Apache 2.0 License.

---

*Ready for submission to open silicon shuttles using 100% open-source EDA tools.*
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