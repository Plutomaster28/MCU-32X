# MCU-32X Caravel Integration

This directory contains the files needed to integrate MCU-32X with the Caravel harness for Efabless Open MPW submission.

## Files

- `user_project_wrapper.v` - Caravel wrapper for MCU-32X
- `config_caravel.tcl` - OpenLane config for Caravel integration
- `README.md` - This file

## Caravel Integration Overview

The Caravel harness provides:
- **Padframe** with 38 GPIO pins
- **Management SoC** (RISC-V processor)
- **Wishbone bus** for communication
- **Power distribution**
- **Clock management**

## MCU-32X Integration

The MCU-32X processor is integrated as follows:

### GPIO Pin Assignment
- `GPIO[7:0]` - ALU result output (result_low)
- `GPIO[15:8]` - Memory address output (address_low)  
- `GPIO[16]` - Memory read enable
- `GPIO[17]` - Memory write enable
- `GPIO[37:18]` - Reserved for future expansion

### Wishbone Interface
The wrapper provides a simple Wishbone slave interface:
- `0x30000000` - Read ALU result
- `0x30000004` - Read memory address
- `0x30000008` - Read memory control signals

### Logic Analyzer
Debug signals are routed to the logic analyzer:
- `LA[7:0]` - ALU result
- `LA[15:8]` - Memory address
- `LA[16]` - Memory read
- `LA[17]` - Memory write

## Setup Instructions

### Prerequisites
```bash
# Install Caravel user project template
git clone https://github.com/efabless/caravel_user_project.git
cd caravel_user_project
git submodule update --init
make install
```

### Integration Steps

1. **Copy MCU-32X files to Caravel project:**
   ```bash
   # Copy source files
   cp -r /path/to/MCU-32X/src caravel_user_project/verilog/rtl/
   
   # Copy integration files
   cp user_project_wrapper.v caravel_user_project/verilog/rtl/
   cp config_caravel.tcl caravel_user_project/openlane/user_project_wrapper/
   ```

2. **Update Caravel includes:**
   ```bash
   cd caravel_user_project
   # Add MCU-32X to the include list
   echo '`include "MCU32X.v"' >> verilog/rtl/user_project_wrapper.v
   ```

3. **Run Caravel hardening:**
   ```bash
   # Harden the user project wrapper
   make user_project_wrapper
   
   # Run full Caravel flow
   make ship
   ```

## Expected Results

After successful integration:
- **User area utilization:** ~20%
- **Clock frequency:** 40MHz (Caravel limitation)
- **GPIO pins used:** 18 out of 38
- **Power consumption:** <1mW estimated

## Submission to Efabless

Once integrated and verified:

1. **Fork the caravel_user_project repository**
2. **Replace the default design with MCU-32X integration**
3. **Update documentation and README**
4. **Submit to Efabless Open MPW program**

## Testing

The integrated design can be tested using:
- **Caravel testbench** - Functional verification
- **Logic analyzer** - Real-time debugging
- **Wishbone access** - Register readback
- **GPIO monitoring** - External signal verification

## Notes

- Clock frequency reduced to 40MHz due to Caravel limitations
- All 32-bit internal buses maintained in MCU-32X
- Future versions can expand GPIO usage
- Wishbone interface allows software control from management core
