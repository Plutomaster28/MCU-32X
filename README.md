# MCU-32X: 32-bit RISC-V Style Processor

![MCU-32X Block Diagram](docs/block_diagram.png)

# MCU-32X: 32-bit RISC-V RV32I Processor

![MCU-32X Block Diagram](docs/block_diagram.png)

## Overview

MCU-32X is a 32-bit RISC-V RV32I processor designed for demonstration and educational purposes. It implements the complete RISC-V RV32I base instruction set with expanded I/O capabilities, designed to be compatible with 180nm process nodes using open-source EDA tools. This design showcases a full CPU implementation from RTL to GDSII with extensive pin connectivity for demonstration purposes.

## Features

- **Full RISC-V RV32I architecture** - Complete base instruction set
- **100MHz target clock frequency** (conservative for 180nm)
- **Pipelined execution** with 5-stage pipeline (Fetch, Decode, Execute, Memory, Writeback)
- **Standard RISC-V register file** - 32 registers with x0 hardwired to zero
- **Expanded I/O interface** - Over 150 pins for demonstration
- **External memory interfaces** - Separate instruction and data memory
- **GPIO capabilities** - 32-bit GPIO with direction control
- **Debug interface** - PC visibility and register inspection
- **Performance counters** - Cycle and instruction counting
- **Interrupt support** - External interrupt lines (framework)
- **Large die area** - 2mm x 2mm for easy visibility and demonstration

## Specifications

| Parameter | Value |
|-----------|-------|
| Architecture | RISC-V RV32I Base ISA |
| Process Node | 180nm compatible (demo using SkyWater 130nm tools) |
| Target Frequency | 100MHz (conservative for demonstration) |
| Die Size | 2mm × 2mm (large for easy demonstration) |
| Core Area | 1.8mm × 1.8mm |
| Core Utilization | 5% (very low for easy routing and demos) |
| Power Grid Pitch | 100µm (horizontal and vertical) |
| Metal Layers Used | met1 through met5 |
| I/O Pins | 150+ (expanded for demonstration) |
| Register File | 32 × 32-bit registers (x0-x31) |
| Memory Interface | 32-bit address, 32-bit data |
| GPIO Pins | 32-bit bidirectional with direction control |

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

### Core Interface
| Pin Name | Direction | Width | Description |
|----------|-----------|-------|-------------|
| clk | Input | 1 | System clock (100MHz) |
| reset | Input | 1 | Active high reset |

### Memory Interface
| Pin Name | Direction | Width | Description |
|----------|-----------|-------|-------------|
| mem_addr[31:0] | Output | 32 | Data memory address |
| mem_wdata[31:0] | Output | 32 | Data memory write data |
| mem_rdata[31:0] | Input | 32 | Data memory read data |
| mem_read | Output | 1 | Data memory read enable |
| mem_write | Output | 1 | Data memory write enable |
| mem_strb[3:0] | Output | 4 | Data memory byte strobes |
| mem_ready | Input | 1 | Data memory ready signal |

### Instruction Memory Interface
| Pin Name | Direction | Width | Description |
|----------|-----------|-------|-------------|
| imem_addr[31:0] | Output | 32 | Instruction memory address |
| imem_rdata[31:0] | Input | 32 | Instruction memory read data |
| imem_read | Output | 1 | Instruction memory read enable |
| imem_ready | Input | 1 | Instruction memory ready signal |

### GPIO Interface
| Pin Name | Direction | Width | Description |
|----------|-----------|-------|-------------|
| gpio_out[31:0] | Output | 32 | GPIO output data |
| gpio_in[31:0] | Input | 32 | GPIO input data |
| gpio_dir[31:0] | Output | 32 | GPIO direction control (1=output, 0=input) |

### Debug Interface
| Pin Name | Direction | Width | Description |
|----------|-----------|-------|-------------|
| pc_out[31:0] | Output | 32 | Current program counter |
| reg_debug[31:0] | Output | 32 | Debug register output |
| reg_debug_addr[4:0] | Output | 5 | Debug register address |
| cpu_halted | Output | 1 | CPU halt status |

### Interrupt Interface
| Pin Name | Direction | Width | Description |
|----------|-----------|-------|-------------|
| irq_lines[7:0] | Input | 8 | External interrupt lines |
| irq_ack | Output | 1 | Interrupt acknowledge |

### Performance Monitoring
| Pin Name | Direction | Width | Description |
|----------|-----------|-------|-------------|
| cycle_count[31:0] | Output | 32 | Cycle counter |
| instr_count[31:0] | Output | 32 | Instruction counter |

**Total I/O Pins: 154** (designed for demonstration and easy interfacing)

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