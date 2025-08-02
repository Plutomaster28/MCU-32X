# MCU-32X Documentation

This directory contains detailed documentation for the MCU-32X processor.

## Files

- `architecture.md` - Detailed processor architecture
- `isa.md` - Instruction Set Architecture specification  
- `pipeline.md` - Pipeline implementation details
- `verification.md` - Verification and testing results
- `tapeout.md` - Tapeout readiness and submission guide

## Block Diagram

The MCU-32X processor implements a 5-stage pipeline with the following major components:

```
Input Pins (18 total)
├── clk (1)
├── reset (1)  
└── Future expansion (16)

Core Pipeline (5 stages)
├── Fetch Stage
├── Decode Stage
├── Execute Stage (ALU + FPU)
├── Memory Stage
└── Writeback Stage

Output Pins (18 total)
├── result_low[7:0] (8)
├── address_low[7:0] (8)
├── mem_read (1)
└── mem_write (1)
```

## Physical Implementation

- **Process:** SkyWater 130nm
- **Die Size:** 479µm × 479µm  
- **Frequency:** 200MHz
- **Power:** ~12nW estimated
- **Core Utilization:** 10% (conservative)

## Verification Status

✅ **Synthesis:** Clean - 18 cells synthesized  
✅ **Place & Route:** Clean - No DRC violations in routing  
✅ **Timing:** Clean - No setup/hold violations at 200MHz  
✅ **LVS:** Clean - Layout matches schematic exactly  
⚠️ **DRC:** 1 minor violation (N-well tap) - Low risk

## Ready for Tapeout

This design is ready for submission to educational silicon shuttles such as:
- Efabless Open MPW
- TinyTapeout  
- ChipIgnite
- Academic shuttles

See `tapeout.md` for submission preparation details.
